"""DNGA longitudinal control: V3.3-style ACC behavior with read-only HEV handoff feedback."""

from dataclasses import dataclass

from numpy import clip, interp
from cereal import messaging
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.dnga.values import BrakeState, CarControllerParams, LongitudinalParams as P
from opendbc.car.dnga.dnga_hybrid_feedback import hybrid_feedback_snapshot


@dataclass
class PlanState:
  fresh: bool
  has_lead: bool
  accel: float
  brake: float


@dataclass
class LeadState:
  visible: bool
  status: bool
  distance: float
  relative_speed: float
  speed: float
  closing_speed: float
  ttc: float
  relevant: bool
  urgent: bool
  stopped: bool


@dataclass
class StopGuard:
  stock_brake: float
  stock_active: bool
  predictive_confirmed: bool
  entry: bool
  request: float
  completion: bool
  active: bool


@dataclass
class BrakeRequest:
  hydraulic: bool
  release_pump: bool
  sng_release: bool
  state: int = BrakeState.DISABLED
  pump: float = 0.0
  magnitude: int = 200


@dataclass
class LongitudinalCommand:
  enabled: bool
  lead: bool
  speed: float
  is_accel: bool
  is_decel: bool
  brake_state: int
  pump: float
  magnitude: int


def normalize_plan_source(source):
  """Return a stable lowercase longitudinalPlan source name."""
  try:
    source = str(source).strip().lower()
  except Exception:
    return ""
  if "." in source:
    source = source.rsplit(".", 1)[-1]
  return source


def distance_profile(distance_val):
  """Return planner-entry, normal brake cap, and lead accel cap.

  CS.op_distance_val follows the local SetDistance enum:
    0 = aggressive / 1 bar
    1 = normal / 2 bars
    2 = far / 3 bars
  """
  try:
    distance_val = int(distance_val)
  except Exception:
    distance_val = 1
  if distance_val <= 0:
    return (0.18, 0.42, 0.12)
  if distance_val >= 2:
    return (0.16, 0.36, 0.08)
  return (0.17, 0.39, 0.1)


def low_speed_brake_cap(v_ego):
  """Taper moving brake authority smoothly toward zero vehicle speed."""
  return float(interp(v_ego, [0.15, 0.3, 0.6, 1.0, 1.5, 2.5], [0.05, 0.08, 0.15, 0.25, 0.4, 0.75]))


def high_speed_brake_cap(v_ego):
  """Keep hydraulic authority conservative at road/highway speed."""
  return float(interp(v_ego, [0.0, 15.0, 25.0, 35.0, 45.0], [0.65, 0.65, 0.6, 0.52, 0.48]))


def progressive_hydraulic_entry(v_ego):
  return float(interp(v_ego, [8.0, 15.0, 25.0, 35.0], [P.HIGHWAY_ENTRY_MIN, 0.14, 0.17, P.HIGHWAY_ENTRY_MAX]))


def progressive_brake_cap(v_ego):
  return float(interp(v_ego, [8.0, 15.0, 25.0, 35.0], [P.HIGHWAY_BRAKE_CAP_MIN, 0.27, 0.29, P.HIGHWAY_BRAKE_CAP_MAX]))


def powertrain_decel_cap(v_ego):
  """Maximum below-vEgo 0x273 offset used for smooth lead deceleration."""
  return float(interp(v_ego, [0.0, 4.0, 15.0, 25.0, 35.0], [0.08, 0.15, 0.28, 0.40, 0.45]))


def encode_hev_brake(brake_cmd):
  """Return (negative pump reaction, legacy combined raw magnitude)."""
  brake_cmd = float(clip(brake_cmd, P.BRAKE_MIN, P.STOP_BRAKE_MAX))
  pump = 0.5 if brake_cmd >= P.PUMP_05_THRESHOLD else 0.4
  magnitude_byte = int(round(200.0 - 100.0 * brake_cmd))
  magnitude_byte = int(clip(magnitude_byte, 0, 255))
  pump_byte = int(round(pump * 10.0))
  combined_magnitude = pump_byte << 8 | magnitude_byte
  return (-pump, combined_magnitude)


def relative_stop_request(v_ego, d_rel, closing_speed):
  """Bounded relative-motion request for a trusted slow/stopping lead.

  This is a fallback when the passive factory request is unavailable. It uses
  only the closing energy inside the remaining gap; it does not infer regen,
  friction pressure, or positive drive torque.
  """
  usable_distance = max(0.5, d_rel - P.PREDICTIVE_STANDSTILL_GAP - P.PREDICTIVE_REACTION_TIME * max(0.0, v_ego))
  relative_decel = closing_speed * closing_speed / (2.0 * usable_distance)
  return float(clip(relative_decel, P.PREDICTIVE_INITIAL_BRAKE, P.STOP_BRAKE_MAX))


@dataclass
class SessionState:
  allowed: bool
  enabled: bool
  gas_override: bool
  feedback: dict
  feedback_clean: bool


@dataclass
class PropulsionState:
  """This cycle's decisions, retained while brake and overshoot latches update."""

  positive_agreement: bool = False
  departing_lead: bool = False
  distant_nonclosing_lead: bool = False
  lead_nonblocking: bool = False
  release_lead: bool = False
  engagement_guard: bool = False
  ramp_ready: bool = False
  blocked: bool = False
  accel: float = 0.0
  lead_confirmed: bool = False
  low_speed_request: bool = False
  arm_active: bool = False
  arm_complete: bool = False
  accel_cap: float = 0.0


class LongitudinalController:
  def __init__(self):
    self.prev_enabled = False
    self._reset(0, 0.0)
    self.plan_source = ""
    self.plan_accel = 0.0
    self.plan_accel_next = 0.0
    self.plan_has_lead = False
    self.plan_frame = -1000000
    self.lead0_status = False
    self.lead0_drel = 0.0
    self.lead0_vrel = 0.0
    self.lead1_status = False
    self.lead1_drel = 0.0
    self.lead1_vrel = 0.0
    self.radar_frame = -1000000
    try:
      self.plan_sm = messaging.SubMaster(["longitudinalPlan"])
    except Exception:
      self.plan_sm = None
    try:
      self.radar_sm = messaging.SubMaster(["radarState"])
    except Exception:
      self.radar_sm = None

  def _reset(self, frame, a_ego):
    """Reset engagement-local longitudinal state; subscriber history survives."""
    self.block_brake_until_frame = frame
    self.apply_brake = 0.0
    self.brake_target = 0.0
    self.brake_active = False
    self.brake_entry_counter = 0
    self.urgent_entry_counter = 0
    self.sng_armed = False
    self.stop_hold = False
    self.sng_release_count = 0
    self.lead_counter = 0
    self.brake_reentry_frame = frame
    self.speed_offset = 0.0
    self.release_counter = 0
    self.lead_accel_counter = 0
    self.lead_loss_counter = 0
    self.urgent_brake = False
    self.handoff_counter = 0
    self.handoff_active = False
    self.handoff_pending = False
    self.handoff_ready_counter = 0
    self.low_speed_neutral_until_frame = frame
    self.low_speed_arm_start_frame = -1000000
    self.low_speed_overshoot_block_until_frame = frame
    self.low_speed_guard_until_frame = frame
    self.stopped_lead_counter = 0
    self.hold_resume_counter = 0
    self.release_lead_until_frame = frame
    self.overshoot_counter = 0
    self.overshoot_block_until_frame = frame
    self.filtered_aego = a_ego
    self.release_pump_until_frame = frame
    self.predictive_entry_counter = 0
    self.stop_guard_latched = False

  def _start_staged_release(self, frame):
    """Start stock-observed release framing and feedback-gated propulsion handoff."""
    self.handoff_pending = True
    self.handoff_ready_counter = 0
    self.release_pump_until_frame = max(self.release_pump_until_frame, frame + P.RELEASE_PUMP_FRAMES)
    self.release_lead_until_frame = max(self.release_lead_until_frame, frame + P.RELEASE_LEAD_HOLD_FRAMES)
    self.speed_offset = min(0.0, self.speed_offset)

  def _clear_hydraulic(self, frame, reentry=True):
    self.apply_brake = 0.0
    self.brake_target = 0.0
    self.brake_active = False
    self.brake_entry_counter = 0
    self.urgent_entry_counter = 0
    self.release_counter = 0
    self.urgent_brake = False
    self.handoff_counter = 0
    self.handoff_active = False
    self.predictive_entry_counter = 0
    self.stop_guard_latched = False
    if reentry:
      self.brake_reentry_frame = frame + P.REENTRY_BLOCK_FRAMES

  def _update_messages(self, frame):
    if self.plan_sm is not None:
      try:
        self.plan_sm.update(0)
        if self.plan_sm.updated["longitudinalPlan"]:
          long_plan = self.plan_sm["longitudinalPlan"]
          self.plan_source = normalize_plan_source(getattr(long_plan, "longitudinalPlanSource", ""))
          self.plan_has_lead = bool(getattr(long_plan, "hasLead", False))
          plan_accels = getattr(long_plan, "accels", [])
          if len(plan_accels) > 0:
            self.plan_accel = float(plan_accels[0])
            self.plan_accel_next = float(plan_accels[1]) if len(plan_accels) > 1 else self.plan_accel
          self.plan_frame = frame
      except Exception:
        pass
    if self.radar_sm is not None:
      try:
        self.radar_sm.update(0)
        if self.radar_sm.updated["radarState"]:
          radar_state = self.radar_sm["radarState"]
          lead0 = radar_state.leadOne
          lead1 = radar_state.leadTwo
          self.lead0_status = bool(lead0.status)
          self.lead0_drel = float(lead0.dRel)
          self.lead0_vrel = float(lead0.vRel)
          self.lead1_status = bool(lead1.status)
          self.lead1_drel = float(lead1.dRel)
          self.lead1_vrel = float(lead1.vRel)
          self.radar_frame = frame
      except Exception:
        pass

  def _get_plan(self, frame, apply_accel):
    # The MPC may select cruise as its limiting trajectory while hasLead is true.
    plan_fresh = frame - self.plan_frame <= P.PLAN_MAX_AGE_FRAMES
    planner_source_lead = plan_fresh and self.plan_source in ("lead0", "lead1")
    planner_reports_lead = plan_fresh and (self.plan_has_lead or planner_source_lead)
    planner_accel_request = 0.7 * self.plan_accel + 0.3 * self.plan_accel_next if plan_fresh else apply_accel
    planner_brake_request = max(0.0, -planner_accel_request)
    return PlanState(plan_fresh, planner_reports_lead, planner_accel_request, planner_brake_request)

  def _update_lead(self, CS, frame, lead, plan):
    radar_fresh = frame - self.radar_frame <= P.RADAR_MAX_AGE_FRAMES
    if self.plan_source == "lead1" and self.lead1_status:
      selected_lead_status = True
      selected_lead_drel = self.lead1_drel
      selected_lead_vrel = self.lead1_vrel
    else:
      selected_lead_status = self.lead0_status
      selected_lead_drel = self.lead0_drel
      selected_lead_vrel = self.lead0_vrel
    if not radar_fresh:
      selected_lead_status = bool(lead)
      selected_lead_drel = 0.0
      selected_lead_vrel = 0.0
    closing_speed = max(0.0, -selected_lead_vrel)
    ttc = selected_lead_drel / closing_speed if selected_lead_status and closing_speed > 0.1 else 999.0
    selected_lead_speed = max(0.0, CS.out.vEgo + selected_lead_vrel)
    if lead:
      self.lead_counter = min(self.lead_counter + 1, P.LEAD_TRUST_COUNT)
    else:
      self.lead_counter = 0
    trusted_lead = self.lead_counter >= P.LEAD_TRUST_COUNT
    urgent_lead = self.lead_counter >= P.URGENT_LEAD_COUNT
    if lead:
      self.lead_loss_counter = 0
    else:
      self.lead_loss_counter = min(self.lead_loss_counter + 1, P.LEAD_LOSS_COUNT)
    relevant_lead = trusted_lead and plan.has_lead if plan.fresh else trusted_lead
    relevant_urgent_lead = urgent_lead and plan.has_lead if plan.fresh else urgent_lead
    stopped_lead_candidate = (
      selected_lead_status
      and bool(lead)
      and (plan.has_lead if plan.fresh else True)
      and (P.STOP_HOLD_MIN_DISTANCE <= selected_lead_drel <= P.STOP_HOLD_MAX_DISTANCE)
      and (selected_lead_speed <= P.STOPPED_LEAD_MAX_SPEED)
    )
    if stopped_lead_candidate:
      self.stopped_lead_counter = min(self.stopped_lead_counter + 1, P.STOP_LEAD_TRUST_COUNT)
    else:
      self.stopped_lead_counter = 0
    trusted_stopped_lead = self.stopped_lead_counter >= P.STOP_LEAD_TRUST_COUNT
    return LeadState(
      bool(lead),
      selected_lead_status,
      selected_lead_drel,
      selected_lead_vrel,
      selected_lead_speed,
      closing_speed,
      ttc,
      relevant_lead,
      relevant_urgent_lead,
      trusted_stopped_lead,
    )

  def _update_session(self, enabled, CS, frame, pcm_cancel_cmd):
    # ACC session/HUD state is independent of 0x275-family feedback.
    # Driver gas suspends actuation but keeps the visible SET session alive.
    session_enabled = enabled and CS.out.cruiseState.enabled
    control_allowed = (
      session_enabled
      and (not pcm_cancel_cmd)
      and (not CS.out.gasPressed)
      and (not CS.out.brakePressed)
    )
    feedback = hybrid_feedback_snapshot(CS, frame)
    feedback_clean = feedback["fresh"] and feedback["consistent"]
    gas_override = session_enabled and CS.out.gasPressed

    return SessionState(control_allowed, session_enabled, gas_override, feedback, feedback_clean)

  def _update_stop_guard(self, CS, frame, planner_brake, moving_allowed, lead_state):
    """Use validated camera braking or planner-confirmed closing geometry as a brake-only floor."""
    stock_brake_rx_frame = int(getattr(CS, "stock_acc_brake_rx_frame", -1000000))
    stock_acc_rx_frame = int(getattr(CS, "stock_acc_request_rx_frame", -1000000))
    stock_brake_fresh = 0 <= frame - stock_brake_rx_frame <= P.STOCK_FRAME_MAX_AGE
    stock_acc_fresh = 0 <= frame - stock_acc_rx_frame <= P.STOCK_FRAME_MAX_AGE
    stock_brake_request = float(clip(getattr(CS, "stock_acc_brake_decel", 0.0), 0.0, P.STOP_BRAKE_MAX))
    stock_brake_pair_valid = (
      stock_brake_fresh
      and stock_acc_fresh
      and (int(getattr(CS, "stock_acc_brake_state", 0)) == BrakeState.BRAKING)
      and bool(getattr(CS, "stock_acc_request_enabled", False))
      and bool(getattr(CS, "stock_acc_request_lead", False))
      and bool(getattr(CS, "stock_acc_request_is_decel", False))
      and (not bool(getattr(CS, "stock_acc_request_is_accel", False)))
      and (stock_brake_request >= P.STOP_GUARD_MIN_STOCK_BRAKE)
    )
    stock_brake_context = (
      moving_allowed
      and stock_brake_pair_valid
      and lead_state.relevant
      and lead_state.status
      and (CS.out.vEgo <= P.STOP_GUARD_MAX_SPEED)
      and (0.0 < lead_state.distance <= P.STOP_GUARD_MAX_DISTANCE)
      and (lead_state.closing_speed >= P.STOP_GUARD_MIN_CLOSING)
    )
    # A fresh, checksum-valid stock camera brake pair is already direct brake
    # evidence. Do not veto it with the downstream longitudinal PID.
    stock_brake_entry = stock_brake_context
    stock_brake_guard = stock_brake_context
    predictive_stop_context = (
      moving_allowed
      and lead_state.relevant
      and lead_state.status
      and (CS.out.vEgo <= P.STOP_GUARD_MAX_SPEED)
      and (0.0 < lead_state.distance <= P.STOP_GUARD_MAX_DISTANCE)
      and (lead_state.closing_speed >= P.PREDICTIVE_MIN_CLOSING)
      and (lead_state.ttc <= P.PREDICTIVE_MAX_TTC)
      and (lead_state.speed <= P.PREDICTIVE_MAX_LEAD_SPEED)
    )
    # The radar/model fallback is not direct actuator evidence, so require
    # matching negative planner intent rather than the downstream PID.
    predictive_stop_entry = predictive_stop_context and planner_brake >= P.PREDICTIVE_MIN_PLANNER_BRAKE
    if predictive_stop_entry:
      self.predictive_entry_counter = min(self.predictive_entry_counter + 1, P.PREDICTIVE_ENTRY_COUNT)
    else:
      self.predictive_entry_counter = 0
    predictive_stop_confirmed = self.predictive_entry_counter >= P.PREDICTIVE_ENTRY_COUNT
    stop_guard_entry = stock_brake_entry or predictive_stop_confirmed
    relative_brake_request = (
      relative_stop_request(CS.out.vEgo, lead_state.distance, lead_state.closing_speed)
      if predictive_stop_context
      else 0.0
    )
    stop_guard_request = stock_brake_request if stock_brake_guard else relative_brake_request
    stop_completion_guard = (
      self.stop_guard_latched
      and lead_state.relevant
      and lead_state.status
      and (not CS.out.standstill)
      and (CS.out.vEgo <= P.SNG_ARM_SPEED)
      and (0.0 < lead_state.distance <= P.STOP_COMPLETION_MAX_DISTANCE)
      and (lead_state.speed <= P.STOP_COMPLETION_MAX_LEAD_SPEED)
    )
    stop_guard_authority = stock_brake_guard or predictive_stop_context or stop_completion_guard
    return StopGuard(
      stock_brake_request,
      stock_brake_guard,
      predictive_stop_confirmed,
      stop_guard_entry,
      stop_guard_request,
      stop_completion_guard,
      stop_guard_authority,
    )

  def _update_stop_hold(self, CS, frame, session, apply_accel, plan, lead_state):
    # Resume requires sustained lead motion and positive demand; PID alone
    # cannot release a stopped-lead hold while the fresh planner asks to brake.
    if self.stop_hold:
      hold_safety_exit = not session.allowed
      hold_resume_candidate = (
        lead_state.status
        and bool(lead_state.visible)
        and (plan.has_lead if plan.fresh else True)
        and (lead_state.speed >= P.HOLD_RESUME_LEAD_SPEED)
        and (plan.accel >= P.SNG_RELEASE_ACCEL if plan.fresh else apply_accel >= P.SNG_RELEASE_ACCEL)
      )
      if hold_resume_candidate:
        self.hold_resume_counter = min(self.hold_resume_counter + 1, P.HOLD_RESUME_COUNT)
      else:
        self.hold_resume_counter = 0
      hold_resume = self.hold_resume_counter >= P.HOLD_RESUME_COUNT
      if hold_safety_exit or hold_resume:
        self.stop_hold = False
        self.hold_resume_counter = 0
        self._clear_hydraulic(frame, reentry=False)
        if hold_safety_exit:
          self.sng_armed = False
          self.sng_release_count = 0
          self.release_lead_until_frame = frame
        else:
          self.sng_release_count = P.SNG_RELEASE_COUNT
          self._start_staged_release(frame)
      else:
        self.brake_active = True
        self.apply_brake = P.SNG_HOLD_BRAKE
        self.brake_target = P.SNG_HOLD_BRAKE
        self.speed_offset = 0.0
    sng_release_active = False
    if self.sng_release_count > 0:
      if not session.allowed:
        self.sng_release_count = 0
        self.sng_armed = False
      else:
        sng_release_active = True
        self.sng_release_count -= 1
        self.brake_active = False
        self.apply_brake = 0.0
        self.brake_target = 0.0
        self.speed_offset = 0.0
    planner_allows_hold = plan.accel < P.SNG_RELEASE_ACCEL if plan.fresh else apply_accel < P.SNG_RELEASE_ACCEL
    direct_standstill_hold = session.allowed and CS.out.standstill and lead_state.stopped and planner_allows_hold
    approached_standstill_hold = self.sng_armed and CS.out.standstill and session.allowed and planner_allows_hold
    if not sng_release_active and (not self.stop_hold) and (direct_standstill_hold or approached_standstill_hold):
      self.stop_hold = True
      self.sng_armed = True
      self.brake_active = True
      self.apply_brake = P.SNG_HOLD_BRAKE
      self.brake_target = P.SNG_HOLD_BRAKE
      self.speed_offset = 0.0
      self.release_lead_until_frame = max(self.release_lead_until_frame, frame + P.RELEASE_LEAD_HOLD_FRAMES)
    return sng_release_active

  def _apply_brake_target(self, raw_target_brake, brake_floor, brake_cap, emergency_closing, critical_closing, guard):
    if guard.active:
      filter_up = P.STOP_BRAKE_FILTER_UP
      step_up = P.STOP_BRAKE_STEP_UP
    elif emergency_closing:
      filter_up = P.EMERGENCY_BRAKE_FILTER_UP
      step_up = P.EMERGENCY_BRAKE_STEP_UP
    else:
      filter_up = P.BRAKE_FILTER_UP
      step_up = P.BRAKE_STEP_UP_URGENT if self.urgent_brake else P.BRAKE_STEP_UP
    filter_alpha = filter_up if raw_target_brake > self.brake_target else P.BRAKE_FILTER_DOWN
    self.brake_target += filter_alpha * (raw_target_brake - self.brake_target)
    self.brake_target = float(clip(self.brake_target, brake_floor, brake_cap))
    if self.brake_target > self.apply_brake:
      rate_limited_brake = min(self.brake_target, self.apply_brake + step_up)
    else:
      rate_limited_brake = max(self.brake_target, self.apply_brake - P.BRAKE_STEP_DOWN)
    # Apply the measured-deceleration governor after filtering so its reduction
    # is not attenuated a second time. The stop guard still obeys its rise limit.
    if (
      self.filtered_aego <= P.DECEL_GOVERNOR_START
      and (not critical_closing)
      and (not emergency_closing)
      and (not guard.active)
    ):
      rate_limited_brake = min(rate_limited_brake, max(brake_floor, self.apply_brake - P.DECEL_GOVERNOR_STEP_DOWN))
    if guard.active:
      bounded_guard_floor = min(guard.request, self.apply_brake + P.STOP_BRAKE_STEP_UP)
      rate_limited_brake = max(rate_limited_brake, bounded_guard_floor)
    self.apply_brake = float(clip(rate_limited_brake, brake_floor, brake_cap))

  def _update_hydraulic(self, CS, frame, session, apply_accel, plan, lead_state):
    distance_val = int(clip(getattr(CS, "op_distance_val", 1), 0, 2))
    lead_entry_planner, lead_normal_cap, _ = distance_profile(distance_val)
    lead_hydraulic_entry = lead_entry_planner
    if CS.out.vEgo >= P.HIGHWAY_MIN_SPEED:
      lead_hydraulic_entry = max(lead_entry_planner, progressive_hydraulic_entry(CS.out.vEgo))
    emergency_closing = (
      lead_state.relevant
      and lead_state.status
      and (lead_state.closing_speed >= P.EMERGENCY_CLOSING_SPEED)
      and (lead_state.ttc <= P.EMERGENCY_TTC)
      and (plan.brake >= P.EMERGENCY_PLANNER_BRAKE)
    )
    urgent_closing = (
      lead_state.urgent
      and (
        plan.brake >= P.URGENT_PLANNER_DECEL
        or (lead_state.closing_speed >= P.URGENT_CLOSING_SPEED and lead_state.ttc <= P.URGENT_TTC)
      )
      or emergency_closing
    )
    stopped_lead_approach = (
      lead_state.relevant
      and lead_state.status
      and (CS.out.vEgo <= P.SNG_ARM_SPEED)
      and (0.0 < lead_state.distance <= P.STOP_LEAD_MAX_DISTANCE)
      and (lead_state.speed <= P.STOP_LEAD_MAX_SPEED)
      and (lead_state.closing_speed >= P.STOP_LEAD_MIN_CLOSING)
    )
    stop_completion_active = (
      self.sng_armed
      and self.brake_active
      and lead_state.relevant
      and lead_state.status
      and (not CS.out.standstill)
      and (CS.out.vEgo <= P.STOP_COMPLETION_MAX_EGO_SPEED)
      and (0.0 < lead_state.distance <= P.CRAWL_MAX_LEAD_DISTANCE)
      and (lead_state.speed <= P.CRAWL_MAX_LEAD_SPEED)
    )
    creep_stop_guard = (
      lead_state.stopped
      and (not CS.out.standstill)
      and (P.CREEP_GUARD_MIN_EGO < CS.out.vEgo <= P.CREEP_GUARD_MAX_EGO)
      and (lead_state.closing_speed >= P.CREEP_GUARD_MIN_CLOSING)
    )
    early_highway_entry = (
      lead_state.relevant
      and lead_state.status
      and (CS.out.vEgo >= P.EARLY_HIGHWAY_MIN_SPEED)
      and (lead_state.closing_speed >= P.EARLY_HIGHWAY_CLOSING)
      and (lead_state.ttc <= P.EARLY_HIGHWAY_TTC)
      and (lead_state.distance <= max(35.0, CS.out.vEgo * P.EARLY_HIGHWAY_TIME_GAP))
      and (plan.brake >= P.EARLY_HIGHWAY_PLANNER_BRAKE)
    )
    self.filtered_aego += P.AEGO_FILTER_ALPHA * (float(CS.out.aEgo) - self.filtered_aego)
    critical_closing = (
      lead_state.status
      and lead_state.closing_speed >= P.DECEL_GOVERNOR_CRITICAL_CLOSING
      and (lead_state.ttc <= P.DECEL_GOVERNOR_CRITICAL_TTC)
    )
    moving_allowed = session.allowed and (not CS.out.standstill)
    brake_request = max(0.0, -apply_accel)
    guard = self._update_stop_guard(CS, frame, plan.brake, moving_allowed, lead_state)
    sng_release_active = self._update_stop_hold(CS, frame, session, apply_accel, plan, lead_state)
    soft_releasing_hydraulic = False
    if not sng_release_active and (not self.stop_hold) and self.brake_active:
      safety_hard_release = not moving_allowed
      handoff_candidate = (
        CS.out.vEgo >= P.HIGHWAY_MIN_SPEED
        and plan.fresh
        and lead_state.relevant
        and (not urgent_closing)
        and (not self.urgent_brake)
        and (not self.sng_armed)
        and (not stop_completion_active)
        and (plan.brake < lead_hydraulic_entry)
        and (apply_accel >= P.HANDOFF_PID_ACCEL)
        and (CS.out.aEgo <= P.HANDOFF_AEGO_MAX)
      )
      if handoff_candidate and (not self.handoff_active):
        self.handoff_counter = min(self.handoff_counter + 1, P.HANDOFF_COUNT)
        if self.handoff_counter >= P.HANDOFF_COUNT:
          self.handoff_active = True
      elif not self.handoff_active:
        self.handoff_counter = 0
      if self.lead_loss_counter >= P.LEAD_LOSS_COUNT:
        safety_hard_release = True
      elif plan.fresh and (not plan.has_lead):
        self.sng_armed = False
        safety_hard_release = True
      hold_brake_to_standstill = (
        self.sng_armed
        and (stopped_lead_approach or stop_completion_active)
        and (CS.out.vEgo <= P.SNG_ARM_SPEED)
        or guard.completion
      )
      low_demand = (
        False
        if hold_brake_to_standstill or guard.active
        else plan.brake < P.PLANNER_RELEASE
        if plan.fresh
        else brake_request < 0.12
      )
      if safety_hard_release:
        self.sng_armed = False
        if session.allowed:
          self._start_staged_release(frame)
        self._clear_hydraulic(frame)
      elif self.handoff_active or low_demand:
        self.release_counter = min(self.release_counter + 1, P.RELEASE_CONFIRM_COUNT)
        soft_releasing_hydraulic = True
        self.brake_target = P.BRAKE_MIN
        release_step = P.HANDOFF_STEP_DOWN if self.handoff_active else P.BRAKE_STEP_DOWN
        self.apply_brake = max(0.0, self.apply_brake - release_step)
        if self.release_counter >= P.RELEASE_CONFIRM_COUNT and self.apply_brake <= P.BRAKE_MIN:
          self.sng_armed = False
          self._start_staged_release(frame)
          self._clear_hydraulic(frame)
      else:
        self.release_counter = 0
    if not sng_release_active and (not self.stop_hold) and (not self.brake_active):
      stopped_lead_reentry = (
        stopped_lead_approach and plan.brake >= P.STOP_LEAD_MIN_BRAKE or creep_stop_guard or guard.entry
      )
      lead_entry = (
        moving_allowed
        and (lead_state.relevant or creep_stop_guard)
        and (plan.brake >= lead_hydraulic_entry or stopped_lead_reentry or early_highway_entry or guard.entry)
        and (CS.out.vEgo > (P.CREEP_GUARD_MIN_EGO if creep_stop_guard else P.MIN_ENTRY_SPEED))
        and (frame > self.block_brake_until_frame or creep_stop_guard)
        and (frame >= self.brake_reentry_frame or stopped_lead_reentry or guard.entry)
      )
      urgent_entry = (
        moving_allowed
        and urgent_closing
        and (plan.brake >= 0.25 or emergency_closing)
        and (CS.out.vEgo > P.MIN_ENTRY_SPEED)
        and (frame > self.block_brake_until_frame)
      )
      if lead_entry:
        required_entry_count = 1 if guard.entry else P.CREEP_ENTRY_COUNT if creep_stop_guard else P.LEAD_ENTRY_COUNT
        self.brake_entry_counter = min(self.brake_entry_counter + 1, required_entry_count)
      else:
        required_entry_count = P.LEAD_ENTRY_COUNT
        self.brake_entry_counter = 0
      if urgent_entry:
        self.urgent_entry_counter = min(self.urgent_entry_counter + 1, P.URGENT_ENTRY_COUNT)
      else:
        self.urgent_entry_counter = 0
      urgent_confirmed = urgent_entry and self.urgent_entry_counter >= P.URGENT_ENTRY_COUNT
      normal_confirmed = self.brake_entry_counter >= required_entry_count
      if urgent_confirmed or normal_confirmed:
        self.brake_active = True
        self.urgent_brake = bool(urgent_confirmed)
        entry_brake = P.BRAKE_MIN
        if guard.stock_active:
          entry_brake = min(P.STOCK_INITIAL_BRAKE_MAX, max(P.BRAKE_MIN, guard.stock_brake))
        elif guard.predictive_confirmed:
          entry_brake = P.PREDICTIVE_INITIAL_BRAKE
        self.apply_brake = entry_brake
        self.brake_target = entry_brake
        self.stop_guard_latched = bool(guard.entry)
        self.brake_entry_counter = 0
        self.urgent_entry_counter = 0
        self.release_counter = 0
        self.speed_offset = 0.0
        if creep_stop_guard:
          self.sng_armed = True
          self.apply_brake = max(self.apply_brake, P.CREEP_BRAKE_FLOOR)
          self.brake_target = max(self.brake_target, P.CREEP_BRAKE_FLOOR)
    if self.brake_active and (guard.stock_active or guard.predictive_confirmed):
      self.stop_guard_latched = True
    if self.brake_active and (not self.stop_hold):
      if not soft_releasing_hydraulic:
        speed_scale = interp(CS.out.vEgo, [0.0, 140.0 * CV.KPH_TO_MS], [1.0, 1.0 / 1.5])
        if urgent_closing or plan.brake >= P.URGENT_HARD_DECEL:
          self.urgent_brake = True
        if guard.active:
          requested_cap = P.STOP_BRAKE_MAX
        elif self.urgent_brake:
          requested_cap = P.URGENT_BRAKE_MAX
        elif CS.out.vEgo >= P.HIGHWAY_MIN_SPEED:
          requested_cap = min(lead_normal_cap, progressive_brake_cap(CS.out.vEgo))
        else:
          requested_cap = lead_normal_cap
        brake_cap = min(requested_cap, low_speed_brake_cap(CS.out.vEgo), high_speed_brake_cap(CS.out.vEgo))
        if guard.active:
          # The stop guard has its own measured envelope; do not double-cap it.
          brake_cap = P.STOP_BRAKE_MAX
        elif self.stop_guard_latched:
          brake_cap = max(brake_cap, min(P.STOP_BRAKE_MAX, self.apply_brake))
        crawl_floor_active = self.brake_active and (creep_stop_guard or stop_completion_active or guard.completion)
        brake_floor = max(P.CREEP_BRAKE_FLOOR, P.STOP_COMPLETION_BRAKE_FLOOR) if crawl_floor_active else P.BRAKE_MIN
        if crawl_floor_active:
          brake_cap = max(brake_cap, brake_floor)
        if plan.fresh:
          target_request = plan.brake
          if apply_accel < 0.0:
            pid_extra = max(0.0, brake_request - plan.brake)
            target_request += min(P.PID_BRAKE_ALLOWANCE, P.PID_BRAKE_BLEND * pid_extra)
        else:
          target_request = brake_request
        if guard.active:
          target_request = max(target_request, guard.request, P.CREEP_BRAKE_FLOOR if guard.completion else P.BRAKE_MIN)
        raw_target_brake = float(clip(target_request * speed_scale, brake_floor, brake_cap))
        self._apply_brake_target(raw_target_brake, brake_floor, brake_cap, emergency_closing, critical_closing, guard)
      if (
        self.brake_active
        and lead_state.relevant
        and (CS.out.vEgo <= P.SNG_ARM_SPEED)
        and (
          plan.brake >= P.SNG_ARM_BRAKE
          or (stopped_lead_approach and plan.brake >= P.STOP_LEAD_MIN_BRAKE)
          or creep_stop_guard
          or guard.completion
        )
      ):
        self.sng_armed = True
    if (
      self.sng_armed
      and (not self.stop_hold)
      and (self.sng_release_count == 0)
      and (CS.out.vEgo > P.SNG_APPROACH_SPEED)
      and (plan.accel >= P.PLANNER_ACCEL_ENTRY)
      and (not guard.active)
    ):
      self.sng_armed = False
    hydraulic_req = self.brake_active and self.apply_brake >= P.BRAKE_MIN
    release_pump_active = session.allowed and (not hydraulic_req) and (frame < self.release_pump_until_frame)
    return BrakeRequest(hydraulic_req, release_pump_active, sng_release_active)

  def _encode_brake(self, CS, enabled, brake):
    brake.state = BrakeState.READY if enabled else BrakeState.DISABLED
    brake.pump = 0.0
    brake.magnitude = 200
    if brake.hydraulic:
      # Only confirmed standstill emits HOLD; a moving latched hold uses CREEPING.
      if self.stop_hold and CS.out.standstill:
        brake.state = BrakeState.HOLD
        self.apply_brake = P.SNG_HOLD_BRAKE
      elif self.stop_hold:
        brake.state = BrakeState.CREEPING
        self.apply_brake = max(self.apply_brake, P.CREEP_BRAKE_FLOOR)
        self.brake_target = max(self.brake_target, P.CREEP_BRAKE_FLOOR)
      elif self.brake_active and self.sng_armed and (CS.out.vEgo <= P.SNG_APPROACH_SPEED):
        brake.state = BrakeState.CREEPING
      else:
        brake.state = BrakeState.BRAKING
      brake.pump, brake.magnitude = encode_hev_brake(self.apply_brake)
    elif brake.sng_release or brake.release_pump:
      brake.state = BrakeState.READY
      brake.pump = -0.4
      brake.magnitude = 4 << 8 | 200

  def _update_handoff_feedback(self, session, propulsion, brake):
    """Use HEV feedback only when crossing from deceleration to propulsion."""
    decel_commanded = (
      brake.hydraulic
      or brake.sng_release
      or (propulsion.accel <= -P.DECEL_DEADBAND)
      or (self.speed_offset < -P.SPEED_OFFSET_EPS)
    )
    if decel_commanded:
      self.handoff_pending = True
      self.handoff_ready_counter = 0

    if not session.enabled:
      self.handoff_pending = False
      self.handoff_ready_counter = 0
      propulsion.ramp_ready = True
      return

    ready_candidate = (
      self.handoff_pending
      and (not brake.hydraulic)
      and (not brake.sng_release)
      and (propulsion.accel > -P.DECEL_DEADBAND)
      and (self.speed_offset >= -P.SPEED_OFFSET_EPS)
      and session.feedback_clean
      and session.feedback["brakes_clear"]
      and session.feedback["torque_ramp_ready"]
    )
    if ready_candidate:
      self.handoff_ready_counter = min(self.handoff_ready_counter + 1, P.HYBRID_READY_COUNT)
    elif self.handoff_pending:
      self.handoff_ready_counter = 0

    if self.handoff_pending and self.handoff_ready_counter >= P.HYBRID_READY_COUNT:
      self.handoff_pending = False
      self.handoff_ready_counter = 0

    propulsion.ramp_ready = not self.handoff_pending

  def _update_low_speed_arm(self, CS, frame, session, propulsion, brake):
    propulsion.low_speed_request = (
      session.allowed
      and (not brake.hydraulic)
      and (not brake.sng_release)
      and propulsion.ramp_ready
      and (not self.stop_hold)
      and (CS.out.vEgo < P.LOW_SPEED_MAX)
      and propulsion.positive_agreement
      and propulsion.lead_nonblocking
    )
    low_speed_arm_eligible = (
      propulsion.low_speed_request
      and frame >= self.low_speed_neutral_until_frame
      and (frame >= self.low_speed_overshoot_block_until_frame)
      and (self.speed_offset >= -P.SPEED_OFFSET_EPS)
    )
    if not low_speed_arm_eligible:
      self.low_speed_arm_start_frame = -1000000
    elif self.low_speed_arm_start_frame < 0:
      self.low_speed_arm_start_frame = frame
    propulsion.arm_active = low_speed_arm_eligible and frame - self.low_speed_arm_start_frame < P.LOW_SPEED_ARM_FRAMES
    propulsion.arm_complete = low_speed_arm_eligible and (not propulsion.arm_active)

  def _check_overshoot(self, CS, frame, session, propulsion, lead_state, brake):
    unexpected_positive_accel = (
      session.allowed
      and CS.out.aEgo >= P.OVERSHOOT_AEGO
      and (self.speed_offset <= P.SPEED_OFFSET_EPS)
      and (not propulsion.positive_agreement)
    )
    if unexpected_positive_accel:
      self.overshoot_counter = min(self.overshoot_counter + 1, P.OVERSHOOT_CONFIRM_COUNT)
    else:
      self.overshoot_counter = 0
    if self.overshoot_counter >= P.OVERSHOOT_CONFIRM_COUNT:
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.overshoot_block_until_frame = frame + P.OVERSHOOT_BLOCK_FRAMES
      self.release_lead_until_frame = max(self.release_lead_until_frame, frame + P.RELEASE_LEAD_HOLD_FRAMES)
      self.overshoot_counter = 0
      if lead_state.stopped and CS.out.vEgo <= P.CREEP_GUARD_MAX_EGO:
        self.sng_armed = True
        self.brake_active = True
        self.apply_brake = max(self.apply_brake, P.CREEP_BRAKE_FLOOR)
        self.brake_target = max(self.brake_target, P.CREEP_BRAKE_FLOOR)
        brake.hydraulic = True
    low_speed_overshoot = (
      CS.out.vEgo < P.LOW_SPEED_MAX and self.speed_offset > 0.0 and (CS.out.aEgo >= P.LOW_SPEED_OVERSHOOT_AEGO)
    )
    if low_speed_overshoot:
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      self.low_speed_overshoot_block_until_frame = frame + P.LOW_SPEED_OVERSHOOT_BLOCK_FRAMES
      self.low_speed_neutral_until_frame = max(
        self.low_speed_neutral_until_frame, frame + P.LOW_SPEED_NEUTRAL_DWELL_FRAMES
      )
      self.low_speed_arm_start_frame = -1000000

  def _update_speed_offset(self, CS, frame, session, propulsion, plan, brake):
    t_lookup = 0.35 + 0.07 * CS.out.vEgo

    if not session.allowed:
      self.speed_offset = 0.0
      if not session.enabled:
        self.handoff_pending = False
        self.handoff_ready_counter = 0
      return

    if brake.hydraulic or brake.sng_release:
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      return

    # Known-good V3.3 behavior: trusted-lead decel uses a lowered normal 0x273
    # target before/after hydraulic braking. No-lead negative targets stay off.
    if (
      propulsion.accel <= -P.DECEL_DEADBAND
      and frame > self.block_brake_until_frame
      and frame >= self.brake_reentry_frame
    ):
      target_offset = -min(
        powertrain_decel_cap(CS.out.vEgo),
        max(0.0, -propulsion.accel) * t_lookup,
      )
      if self.speed_offset > 0.0:
        self.speed_offset = 0.0
      elif target_offset < self.speed_offset:
        self.speed_offset = max(target_offset, self.speed_offset - P.DECEL_OFFSET_STEP_DOWN)
      else:
        self.speed_offset = min(target_offset, self.speed_offset + P.DECEL_OFFSET_STEP_UP)
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      return

    # Never cross directly from a negative target to positive propulsion.
    if self.speed_offset < -P.SPEED_OFFSET_EPS:
      self.speed_offset = min(0.0, self.speed_offset + P.TARGET_RETURN_STEP)
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      return
    if self.speed_offset < 0.0:
      self.speed_offset = 0.0

    if (
      propulsion.accel >= P.ACCEL_ENTRY
      and (not propulsion.blocked)
      and (not plan.has_lead or not plan.fresh or propulsion.lead_confirmed or propulsion.low_speed_request)
      and (CS.out.vEgo >= P.LOW_SPEED_MAX or propulsion.arm_complete)
    ):
      if CS.out.vEgo < P.LOW_SPEED_MAX:
        if propulsion.departing_lead:
          accel_cap = max(propulsion.accel_cap, P.DEPARTING_ACCEL_CAP)
          offset_cap = P.DEPARTING_OFFSET_CAP
          accel_step_up = P.DEPARTING_OFFSET_STEP_UP
        else:
          accel_cap = P.LOW_SPEED_ACCEL_CAP
          offset_cap = P.LOW_SPEED_OFFSET_CAP
          accel_step_up = P.LOW_SPEED_OFFSET_STEP_UP
      else:
        accel_cap = propulsion.accel_cap if plan.has_lead and plan.fresh else P.ACCEL_CAP
        offset_cap = 0.3 if plan.has_lead and plan.fresh else P.ACCEL_OFFSET_MAX
        accel_step_up = P.ACCEL_OFFSET_STEP_UP

      target_offset = min(offset_cap, min(propulsion.accel, accel_cap) * t_lookup)
      if target_offset > self.speed_offset:
        self.speed_offset = min(target_offset, self.speed_offset + accel_step_up)
      else:
        self.speed_offset = max(target_offset, self.speed_offset - P.ACCEL_OFFSET_STEP_DOWN)
    elif self.speed_offset > 0.0:
      self.speed_offset = max(0.0, self.speed_offset - P.ACCEL_OFFSET_STEP_DOWN)

  def _build_command(self, CS, frame, session, plan, lead_state, brake, propulsion):
    longitudinal_enabled = session.enabled
    if not longitudinal_enabled:
      brake.state = BrakeState.DISABLED
      brake.pump = 0.0
      brake.magnitude = 200
      des_speed = 0.0
    elif session.gas_override:
      brake.state = BrakeState.READY
      brake.pump = 0.0
      brake.magnitude = 200
      des_speed = CS.out.vEgo
    elif self.stop_hold or brake.sng_release:
      des_speed = 0.0
    elif brake.hydraulic:
      des_speed = CS.out.vEgo
    else:
      des_speed = max(0.0, CS.out.vEgo + self.speed_offset)

    # V3.3 stock-observed 0x273 mode table. Feedback does not own HUD state.
    if not longitudinal_enabled:
      acc_cmd_is_accel = False
      acc_cmd_is_decel = False
    elif brake.state == BrakeState.HOLD:
      acc_cmd_is_accel = True
      acc_cmd_is_decel = True
    elif brake.state in (BrakeState.BRAKING, BrakeState.CREEPING) or brake.sng_release:
      acc_cmd_is_accel = False
      acc_cmd_is_decel = True
    else:
      acc_cmd_is_accel = True
      acc_cmd_is_decel = False

    low_speed_accel_unlock = (
      longitudinal_enabled
      and propulsion.low_speed_request
      and (not brake.hydraulic)
      and (not brake.sng_release)
      and propulsion.ramp_ready
      and (not self.stop_hold)
      and (frame >= self.low_speed_neutral_until_frame)
      and (frame >= self.low_speed_overshoot_block_until_frame)
      and (propulsion.arm_active or self.speed_offset > 0.0)
    )
    lead_for_acc_cmd = bool(
      longitudinal_enabled
      and (lead_state.visible or low_speed_accel_unlock or propulsion.release_lead or self.stop_hold)
    )
    return LongitudinalCommand(
      longitudinal_enabled,
      lead_for_acc_cmd,
      des_speed,
      acc_cmd_is_accel,
      acc_cmd_is_decel,
      brake.state,
      brake.pump,
      brake.magnitude,
    )

  def _update_propulsion(self, CS, frame, session, apply_accel, plan, lead_state, brake):
    propulsion = PropulsionState()
    propulsion.release_lead = session.enabled and frame < self.release_lead_until_frame
    propulsion.engagement_guard = (
      session.allowed and CS.out.vEgo < P.LOW_SPEED_ENGAGEMENT_MAX and (frame < self.low_speed_guard_until_frame)
    )
    propulsion.positive_agreement = plan.fresh and plan.accel >= P.ACCEL_ENTRY and apply_accel > 0.0
    propulsion.departing_lead = (
      lead_state.status
      and lead_state.relative_speed >= P.DEPARTING_LEAD_VREL
      and (lead_state.distance >= P.DEPARTING_LEAD_DREL)
    )
    propulsion.distant_nonclosing_lead = (
      lead_state.status
      and lead_state.distance >= P.NONBLOCKING_LEAD_DISTANCE
      and (lead_state.relative_speed >= P.NONBLOCKING_LEAD_VREL)
    )
    propulsion.lead_nonblocking = (
      not lead_state.status or propulsion.departing_lead or propulsion.distant_nonclosing_lead
    )

    if plan.fresh:
      if plan.accel < 0.0 and lead_state.relevant:
        propulsion.accel = plan.accel
      elif plan.accel > 0.0 and apply_accel > 0.0:
        propulsion.accel = min(plan.accel + 0.08, 0.75 * plan.accel + 0.25 * apply_accel)
      else:
        propulsion.accel = 0.0
    else:
      propulsion.accel = apply_accel if apply_accel >= 0.0 else 0.0

    self._update_handoff_feedback(session, propulsion, brake)
    propulsion.blocked = (
      brake.hydraulic
      or brake.sng_release
      or (not propulsion.ramp_ready)
      or (not plan.fresh)
      or (frame < self.overshoot_block_until_frame)
      or propulsion.engagement_guard
    )

    if plan.has_lead and plan.fresh:
      if plan.accel >= P.PLANNER_ACCEL_ENTRY and apply_accel > 0.0:
        self.lead_accel_counter = min(self.lead_accel_counter + 1, P.LEAD_ACCEL_CONFIRM_COUNT)
      else:
        self.lead_accel_counter = 0
    else:
      self.lead_accel_counter = P.LEAD_ACCEL_CONFIRM_COUNT
    propulsion.lead_confirmed = self.lead_accel_counter >= P.LEAD_ACCEL_CONFIRM_COUNT

    self._update_low_speed_arm(CS, frame, session, propulsion, brake)
    self._check_overshoot(CS, frame, session, propulsion, lead_state, brake)
    propulsion.accel_cap = distance_profile(int(clip(getattr(CS, "op_distance_val", 1), 0, 2)))[2]
    self._update_speed_offset(CS, frame, session, propulsion, plan, brake)
    return self._build_command(CS, frame, session, plan, lead_state, brake, propulsion)

  def update(self, enabled, CS, frame, accel, pcm_cancel_cmd, lead):
    apply_accel = clip(accel, -3.0, 1.5)
    engagement_edge = enabled and (not self.prev_enabled)
    if engagement_edge:
      self._reset(frame, float(CS.out.aEgo))
      self.block_brake_until_frame = frame + 50
      self.low_speed_guard_until_frame = frame + P.LOW_SPEED_ENGAGEMENT_GUARD_FRAMES
    self.prev_enabled = enabled
    if frame % CarControllerParams.ACC_STEP:
      return None
    self._update_messages(frame)
    plan = self._get_plan(frame, apply_accel)
    lead_state = self._update_lead(CS, frame, lead, plan)
    session = self._update_session(enabled, CS, frame, pcm_cancel_cmd)
    # If a new engagement begins while the HEV is visibly still braking or
    # carrying strong negative torque, treat that as a handoff too. This gates
    # only positive target buildup; the ACC session remains enabled.
    if (
      engagement_edge
      and session.feedback_clean
      and ((not session.feedback["brakes_clear"]) or (not session.feedback["torque_ramp_ready"]))
    ):
      self.handoff_pending = True
      self.handoff_ready_counter = 0
    brake = self._update_hydraulic(CS, frame, session, apply_accel, plan, lead_state)
    self._encode_brake(CS, session.enabled, brake)
    return self._update_propulsion(CS, frame, session, apply_accel, plan, lead_state, brake)
