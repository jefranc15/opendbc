#!/usr/bin/env python3
from pathlib import Path
import py_compile

CC_PATH = Path("opendbc/car/dnga/carcontroller.py")
CS_PATH = Path("opendbc/car/dnga/carstate.py")


def replace_once(text, old, new, label):
  count = text.count(old)
  if count != 1:
    raise SystemExit(f"{label}: expected exactly 1 match, found {count}")
  return text.replace(old, new, 1)


cc = CC_PATH.read_text()
cs = CS_PATH.read_text()

# V4.0 header.
cc = replace_once(cc,
'''# V3.3R4 hybrid-feedback-supervisor build: offline/replay/bench validation.\n''',
'''# V4.0 DNGA engagement and hybrid-feedback build.\n#\n# V4.0 combines the R4.1 stock-derived handoff with the 2026-08-29 road-log\n# fixes: SET/RES remains an enabled ACC session during driver gas override,\n# longitudinal fault rearm requires fresh/consistent brake-clear feedback but\n# leaves torque neutrality to the independent propulsion gate, the 0.55-second\n# positive-torque/friction allowance starts at actual physical overlap rather\n# than planner decel intent, and a longitudinal-only hybrid fault no longer\n# tears down otherwise healthy lateral steering.\n''',
"controller header")

# Independent physical-overlap state.
cc = replace_once(cc,
'''    self.v33r4_decel_entry_frame = -1000000\n    self.v33r4_decel_entry_torque = 0\n    self.v33r4_decel_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n''',
'''    self.v33r4_decel_entry_frame = -1000000\n    self.v33r4_decel_entry_torque = 0\n    self.v33r4_decel_torque_cleared = False\n    # V4.0 keeps physical friction/positive-torque overlap timing independent\n    # from the decel-latch entry timestamp. Sharing these states caused the\n    # pre-friction planner interval to leak back into the overlap age.\n    self.v33r4_overlap_entry_frame = -1000000\n    self.v33r4_overlap_entry_torque = 0\n    self.v33r4_overlap_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n''',
"overlap init")

# Longitudinal-only hybrid fault: reset overlap state but keep cruise/lateral latch alive.
cc = replace_once(cc,
'''    self.v33r4_brake_clear_counter = 0\n    self.v33r4_torque_ready_counter = 0\n    self.v33r4_positive_overlap_counter = 0\n    self.v25l_speed_offset = 0.0\n    if hasattr(CS, "is_cruise_latch"):\n      CS.is_cruise_latch = False\n    CS.hybrid_feedback_fault = True\n''',
'''    self.v33r4_brake_clear_counter = 0\n    self.v33r4_torque_ready_counter = 0\n    self.v33r4_overlap_entry_frame = -1000000\n    self.v33r4_overlap_entry_torque = 0\n    self.v33r4_overlap_torque_cleared = False\n    self.v33r4_positive_overlap_counter = 0\n    self.v25l_speed_offset = 0.0\n    # V4.0: hybrid feedback supervises longitudinal actuation only. Keep the\n    # cruise latch and lateral session alive; longitudinal_session_allowed\n    # below still goes false while this fault is latched, so 0x271/0x273 fail\n    # non-propulsive without dropping a healthy 0x1D0 STEER_REQ.\n    CS.hybrid_feedback_fault = True\n''',
"fault isolation")

# Reset independent overlap state on a fresh outer engagement.
cc = replace_once(cc,
'''      self.v33r4_decel_entry_frame = -1000000\n      self.v33r4_decel_entry_torque = 0\n      self.v33r4_decel_torque_cleared = False\n      self.v33r4_positive_overlap_counter = 0\n\n    self.prev_enabled = enabled\n''',
'''      self.v33r4_decel_entry_frame = -1000000\n      self.v33r4_decel_entry_torque = 0\n      self.v33r4_decel_torque_cleared = False\n      self.v33r4_overlap_entry_frame = -1000000\n      self.v33r4_overlap_entry_torque = 0\n      self.v33r4_overlap_torque_cleared = False\n      self.v33r4_positive_overlap_counter = 0\n\n    self.prev_enabled = enabled\n''',
"engagement overlap reset")

# V4.0 session/actuator split. Preserve the Sunnypilot longActive gate.
cc = replace_once(cc,
'''      base_control_allowed = (\n        enabled and\n        long_active and\n        CS.out.cruiseState.enabled and\n        not pcm_cancel_cmd and\n        not CS.out.gasPressed and\n        not CS.out.brakePressed\n      )\n      r4_feedback = hybrid_feedback_snapshot(CS, frame)\n      r4_feedback_clean = (\n        r4_feedback["fresh"] and r4_feedback["consistent"]\n      )\n      r4_rearm_ok = (\n        r4_feedback_clean and\n        r4_feedback["brakes_clear"] and\n        r4_feedback["torque_ramp_ready"] and\n        not r4_feedback["positive_vote"]\n      )\n\n      # A fault is not cleared by timers or by the stale outer enabled flag.\n      # A new SET/RES engagement edge may clear it only while all observed\n      # feedback is fresh, mutually consistent, brake-clear, and non-positive.\n      if engagement_edge and self.v33r4_fault_latched:\n        if r4_rearm_ok:\n          self.v33r4_fault_latched = False\n          self.v33r4_fault_reason = ""\n        else:\n          self._v33r4_latch_fault(CS, "feedback_not_safe_to_rearm")\n\n      if base_control_allowed and not r4_feedback["fresh"]:\n''',
'''      # V4.0 separates the ACC session from actuator authority. Stock accepts\n      # SET/RES and shows the set speed while the driver is overriding with the\n      # accelerator; only brake/CANCEL/session loss disable the 0x271/0x273\n      # session. The Sunnypilot longActive gate is retained. Gas still blocks\n      # every OP brake/propulsion actuator below.\n      base_session_allowed = (\n        enabled and\n        long_active and\n        CS.out.cruiseState.enabled and\n        not pcm_cancel_cmd and\n        not CS.out.brakePressed\n      )\n      base_control_allowed = (\n        base_session_allowed and\n        not CS.out.gasPressed\n      )\n      r4_feedback = hybrid_feedback_snapshot(CS, frame)\n      r4_feedback_clean = (\n        r4_feedback["fresh"] and r4_feedback["consistent"]\n      )\n      # Rearm is a session-level decision, not permission for propulsion. A\n      # clean, brake-clear hybrid state may rearm even while torque is not yet\n      # neutral; r4_torque_ready_candidate remains the independent propulsion\n      # gate after the accelerator is released.\n      r4_rearm_ok = (\n        r4_feedback_clean and\n        r4_feedback["brakes_clear"]\n      )\n\n      # V4.0: because a longitudinal fault no longer destroys the cruise\n      # latch/lateral session, outer `enabled` may stay true. Accept either the\n      # normal outer engagement edge or a real physical SET/RES release edge\n      # from CarState as the explicit driver request to rearm longitudinal.\n      v40_rearm_edge = (\n        engagement_edge or\n        bool(getattr(CS, "v40_acc_rearm_edge", False))\n      )\n      if v40_rearm_edge and self.v33r4_fault_latched:\n        if r4_rearm_ok:\n          self.v33r4_fault_latched = False\n          self.v33r4_fault_reason = ""\n        else:\n          self._v33r4_latch_fault(CS, "feedback_not_safe_to_rearm")\n\n      if base_control_allowed and not r4_feedback["fresh"]:\n''',
"session and rearm")

cc = replace_once(cc,
'''      control_allowed = (\n        base_control_allowed and not self.v33r4_fault_latched\n      )\n      CS.hybrid_feedback_fault = self.v33r4_fault_latched\n''',
'''      control_allowed = (\n        base_control_allowed and not self.v33r4_fault_latched\n      )\n      longitudinal_session_allowed = (\n        base_session_allowed and not self.v33r4_fault_latched\n      )\n      gas_override_active = (\n        longitudinal_session_allowed and CS.out.gasPressed\n      )\n      CS.hybrid_feedback_fault = self.v33r4_fault_latched\n''',
"longitudinal session state")

# R4.1 stock 0x275 release semantics.
cc = replace_once(cc,
'''      def start_v33r_staged_release():\n        # Copy the stock-observed moving release sequence. Keep the pump\n        # reaction at FC/04/C8 and retain deceleration mode for the full\n        # observed 1.2-second pressure-release interval.\n''',
'''      def start_v33r_staged_release():\n        # Copy the stock-observed moving release sequence. Keep FC/04/C8 for\n        # the full observed 1.2-second protocol interval. R4.1 no longer treats\n        # that timer alone as physical braking: 0x273 may arm normal mode and,\n        # after verified neutral torque, ramp propulsion while FC/04/C8 remains.\n''',
"R4.1 release comment")

# Physical overlap timer starts at measured overlap, not planner/decel intent.
old_overlap = '''      # Toyota briefly overlaps positive hybrid torque with brake entry. In\n      # the passive stock capture the longest voted positive-torque + friction\n      # overlap was 0.20 s. Permit a wider 0.55 s entry envelope only while\n      # torque does not rise materially; once torque has cleared, any return of\n      # voted propulsion under friction is a fault.\n      r4_negative_intent = (\n        control_allowed and\n        (\n          hydraulic_req or\n          release_pump_active or\n          self.v33r2_decel_latched or\n          (plan_fresh and planner_brake_request >= V33R2_DECEL_LATCH_BRAKE)\n        )\n      )\n      if r4_negative_intent and self.v33r4_decel_entry_frame < 0:\n        self.v33r4_decel_entry_frame = frame\n        self.v33r4_decel_entry_torque = r4_feedback["torque_actual"]\n        self.v33r4_decel_torque_cleared = (\n          r4_feedback["torque_actual"] <= 80\n        )\n      if r4_negative_intent and r4_feedback["torque_actual"] <= 80:\n        self.v33r4_decel_torque_cleared = True\n\n      r4_positive_under_friction = (\n        control_allowed and\n        r4_feedback_clean and\n        r4_feedback["friction"] > 0 and\n        r4_feedback["positive_vote"]\n      )\n      r4_overlap_age = frame - self.v33r4_decel_entry_frame\n      r4_overlap_rising = (\n        r4_feedback["torque_actual"] >\n        max(80, self.v33r4_decel_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_decel_torque_cleared or\n          r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES or\n          r4_overlap_rising\n        )\n      )\n'''
new_overlap = '''      # Toyota briefly overlaps positive hybrid torque with physical friction\n      # brake entry. The 2026-08-29 R4.1 logs showed the old timer started\n      # 0.5-0.7 s too early from planner/DECEL intent, exhausting the entire\n      # allowance before friction even appeared and falsely dropping control.\n      # V4.0 starts the 0.55 s envelope only when the actual measured overlap\n      # (friction > 0 + positive torque vote) begins. Rising torque, persistence\n      # past the envelope, or positive torque returning after neutral remains a\n      # fault.\n      r4_negative_intent = (\n        control_allowed and\n        (\n          hydraulic_req or\n          release_pump_active or\n          self.v33r2_decel_latched or\n          (plan_fresh and planner_brake_request >= V33R2_DECEL_LATCH_BRAKE)\n        )\n      )\n      r4_positive_under_friction = (\n        control_allowed and\n        r4_feedback_clean and\n        r4_feedback["friction"] > 0 and\n        r4_feedback["positive_vote"]\n      )\n\n      if not r4_negative_intent:\n        self.v33r4_overlap_entry_frame = -1000000\n        self.v33r4_overlap_entry_torque = 0\n        self.v33r4_overlap_torque_cleared = False\n      elif r4_feedback_clean and r4_feedback["torque_actual"] <= 80:\n        # Once the powertrain has crossed through the positive-torque region,\n        # any later return to voted propulsion under friction is not an entry\n        # transient and should fault immediately. Keep this independent from\n        # decel-latch clearing so a release-stage transition cannot reset it.\n        self.v33r4_overlap_torque_cleared = True\n\n      if (\n        r4_positive_under_friction and\n        self.v33r4_overlap_entry_frame < 0\n      ):\n        self.v33r4_overlap_entry_frame = frame\n        self.v33r4_overlap_entry_torque = r4_feedback["torque_actual"]\n\n      r4_overlap_started = self.v33r4_overlap_entry_frame >= 0\n      r4_overlap_age = (\n        frame - self.v33r4_overlap_entry_frame\n        if r4_overlap_started else 0\n      )\n      r4_overlap_rising = (\n        r4_positive_under_friction and\n        r4_overlap_started and\n        r4_feedback["torque_actual"] >\n        max(80, self.v33r4_overlap_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)\n      )\n      r4_overlap_unsafe = (\n        r4_positive_under_friction and\n        (\n          not r4_negative_intent or\n          self.v33r4_overlap_torque_cleared or\n          (\n            r4_overlap_started and\n            r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES\n          ) or\n          r4_overlap_rising\n        )\n      )\n'''
cc = replace_once(cc, old_overlap, new_overlap, "physical overlap timing")

# Timed FC/04/C8 release framing is not itself physical braking.
cc = replace_once(cc,
'''      decel_latch_request = (\n        control_allowed and\n        (\n          hydraulic_req or\n          sng_release_active or\n          release_pump_active or\n          (\n''',
'''      # R4.1: the stock 1.2 s FC/04/C8 stage is protocol framing, not proof\n      # that physical braking is still active. start_v33r_staged_release()\n      # already owns the persistent latch, so the timed release-pump stage must\n      # not continuously re-request/reset that latch.\n      decel_latch_request = (\n        control_allowed and\n        (\n          hydraulic_req or\n          sng_release_active or\n          (\n''',
"decel latch release framing")

cc = replace_once(cc,
'''          not hydraulic_req and\n          not sng_release_active and\n          not release_pump_active and\n          r4_feedback_clean and\n''',
'''          not hydraulic_req and\n          not sng_release_active and\n          r4_feedback_clean and\n''',
"decel clear release framing")

# Reset overlap state whenever control drops.
cc = replace_once(cc,
'''        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_torque_cleared = False\n        self.v33r4_positive_overlap_counter = 0\n        self.v33r2_release_pump_until_frame = frame\n''',
'''        self.v33r4_decel_entry_frame = -1000000\n        self.v33r4_decel_torque_cleared = False\n        self.v33r4_overlap_entry_frame = -1000000\n        self.v33r4_overlap_entry_torque = 0\n        self.v33r4_overlap_torque_cleared = False\n        self.v33r4_positive_overlap_counter = 0\n        self.v33r2_release_pump_until_frame = frame\n''',
"control drop overlap reset")

cc = replace_once(cc,
'''      target_slope_lock = (\n        hydraulic_req or\n        release_pump_active or\n        self.v33r2_decel_latched or\n''',
'''      target_slope_lock = (\n        hydraulic_req or\n        self.v33r2_decel_latched or\n''',
"target slope release framing")

cc = replace_once(cc,
'''        r4_feedback["torque_ramp_ready"] and\n        not hydraulic_req and\n        not release_pump_active and\n        not self.v33r2_decel_latched and\n''',
'''        r4_feedback["torque_ramp_ready"] and\n        not hydraulic_req and\n        not self.v33r2_decel_latched and\n''',
"torque ready release framing")

cc = replace_once(cc,
'''      propulsion_blocked = (\n        hydraulic_req or\n        release_pump_active or\n        self.v33r2_decel_latched or\n''',
'''      # The 1.2 s FC/04/C8 release frame may coexist with positive hybrid\n      # torque in stock. Physical feedback/latch/torque readiness, not the\n      # protocol timer itself, decides whether propulsion may ramp.\n      propulsion_blocked = (\n        hydraulic_req or\n        self.v33r2_decel_latched or\n''',
"propulsion release framing")

cc = replace_once(cc,
'''      # Preserve the existing post-deceleration dwell, now driven by the\n      # persistent latch and verified pump-release stage instead of a negative\n      # 0x273 target. Low speed retains an additional neutral wake delay.\n      regen_or_brake_active = (\n        hydraulic_req or\n        sng_release_active or\n        release_pump_active or\n        self.v33r2_decel_latched\n''',
'''      # R4.1: only physical/latched deceleration extends the neutral dwell.\n      # The stock 1.2 s FC/04/C8 protocol stage can continue after the hybrid\n      # system has already crossed through neutral into positive torque.\n      regen_or_brake_active = (\n        hydraulic_req or\n        sng_release_active or\n        self.v33r2_decel_latched\n''',
"regen dwell release framing")

cc = replace_once(cc,
'''        control_allowed and\n        not hydraulic_req and\n        not sng_release_active and\n        not release_pump_active and\n        not self.v33r2_decel_latched and\n        not self.v25o_stop_hold and\n        CS.out.vEgo < V32R_LOW_SPEED_MAX and\n''',
'''        control_allowed and\n        not hydraulic_req and\n        not sng_release_active and\n        not self.v33r2_decel_latched and\n        not self.v25o_stop_hold and\n        CS.out.vEgo < V32R_LOW_SPEED_MAX and\n''',
"low speed propulsion release framing")

cc = replace_once(cc,
'''      elif (\n        hydraulic_req or\n        sng_release_active or\n        release_pump_active or\n        self.v33r2_decel_latched\n      ):\n        self.v25l_speed_offset = 0.0\n''',
'''      elif (\n        hydraulic_req or\n        sng_release_active or\n        self.v33r2_decel_latched\n      ):\n        self.v25l_speed_offset = 0.0\n''',
"speed offset release framing")

# Final 0x271/0x273 session encoding, including gas override and R4.1 release behavior.
cc = replace_once(cc,
'''      # Longitudinal engagement follows independent physical/cruise override\n      # state, not the stale outer `enabled` bit. This makes gas, brake,\n      # CANCEL, and cruise-latch loss encode an exact disabled command.\n      longitudinal_enabled = control_allowed and self.CP.openpilotLongitudinalControl\n      if not longitudinal_enabled:\n        brake_state = 0x00\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = 0.0\n      elif self.v25o_stop_hold or sng_release_active:\n''',
'''      # Keep the SET/RES ACC session visible through a driver accelerator\n      # override, matching the stock camera. Actuator authority remains\n      # `control_allowed`, so gas cannot produce OP braking or propulsion.\n      longitudinal_enabled = longitudinal_session_allowed and self.CP.openpilotLongitudinalControl\n      if not longitudinal_enabled:\n        brake_state = 0x00\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = 0.0\n      elif gas_override_active:\n        # Stock-like neutral override framing: preserve enabled 0x01/0x273 and\n        # the cluster set speed, but request no OP acceleration/deceleration.\n        brake_state = 0x01\n        pump_reaction = 0.0\n        brake_mag = 200\n        des_speed = CS.out.vEgo\n      elif self.v25o_stop_hold or sng_release_active:\n''',
"longitudinal gas session")

cc = replace_once(cc,
'''      elif (\n        hydraulic_req or\n        release_pump_active or\n        self.v33r2_decel_latched\n      ):\n        # Never combine braking intent with a lowered or positive 0x273 target.\n        des_speed = CS.out.vEgo\n''',
'''      elif (\n        hydraulic_req or\n        self.v33r2_decel_latched\n      ):\n        # Never combine physical/latched braking intent with a positive target.\n        # A timed FC/04/C8 release frame alone is not physical braking.\n        des_speed = CS.out.vEgo\n''',
"desired speed release framing")

cc = replace_once(cc,
'''      if not longitudinal_enabled:\n        acc_cmd_is_accel = False\n        acc_cmd_is_decel = False\n      elif brake_state == 0x30:\n''',
'''      if not longitudinal_enabled:\n        acc_cmd_is_accel = False\n        acc_cmd_is_decel = False\n      elif gas_override_active:\n        # Stock accepts SET while the driver holds the accelerator. Keep the\n        # normal/ACCEL mode bit armed with an exact current-speed target; OP\n        # positive-target authority remains blocked until gas is released and\n        # the R4 torque-ready gate passes.\n        acc_cmd_is_accel = True\n        acc_cmd_is_decel = False\n      elif brake_state == 0x30:\n''',
"gas override mode")

cc = replace_once(cc,
'''      elif brake_state == 0x30:\n        acc_cmd_is_accel = True\n        acc_cmd_is_decel = True\n      elif (\n        brake_state in (0x21, 0x31) or\n        sng_release_active or\n        release_pump_active or\n        self.v33r2_decel_latched or\n''',
'''      elif brake_state == 0x30:\n        acc_cmd_is_accel = True\n        acc_cmd_is_decel = True\n      elif (\n        release_pump_active and\n        not self.v33r2_decel_latched and\n        r4_accel_arm_ready and\n        plan_fresh\n      ):\n        # Stock 2026-08-26 capture: 0x271 stayed 0x01 + FC/04/C8 for 1.20 s,\n        # while 0x273 changed 0x20 -> 0x40 and hybrid torque crossed positive.\n        # Arm normal/ACCEL mode here, but target buildup remains independently\n        # blocked by torque-ready feedback and the retained dwell/ramp gates.\n        acc_cmd_is_accel = True\n        acc_cmd_is_decel = False\n      elif (\n        brake_state in (0x21, 0x31) or\n        sng_release_active or\n        self.v33r2_decel_latched or\n''',
"release pump ACC mode")

cc = replace_once(cc,
'''        longitudinal_enabled and\n        low_speed_propulsion_request and\n        not hydraulic_req and\n        not sng_release_active and\n        not release_pump_active and\n        not self.v33r2_decel_latched and\n''',
'''        longitudinal_enabled and\n        low_speed_propulsion_request and\n        not hydraulic_req and\n        not sng_release_active and\n        not self.v33r2_decel_latched and\n''',
"low speed unlock release framing")

cc = replace_once(cc,
'''          longitudinal_enabled,  # Independently override-gated longitudinal state\n''',
'''          longitudinal_enabled,  # SET/RES session state; gas override stays enabled\n''',
"ACC command comment")

# CarState one-cycle physical SET/RES release edge.
cs = replace_once(cs,
'''    self.is_plus_btn_latch = False\n    self.is_minus_btn_latch = False\n    self.plus_hold_time = 0.0\n''',
'''    self.is_plus_btn_latch = False\n    self.is_minus_btn_latch = False\n    # V4.0 one-cycle SET/RES release edge consumed by CarController when a\n    # longitudinal-only hybrid feedback fault needs explicit driver rearm.\n    self.v40_acc_rearm_edge = False\n    self.plus_hold_time = 0.0\n''',
"CarState rearm state")

cs = replace_once(cs,
'''    minus_button = bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"])\n    plus_button = bool(cp.vl["PCM_BUTTONS"]["RES_PLUS"])\n\n    if self.is_cruise_latch:\n''',
'''    minus_button = bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"])\n    plus_button = bool(cp.vl["PCM_BUTTONS"]["RES_PLUS"])\n\n    # V4.0: previous latch values are updated later in this block, so this is\n    # true for exactly one CarState cycle on the physical SET or RES release.\n    self.v40_acc_rearm_edge = bool(\n      (self.is_plus_btn_latch and not plus_button) or\n      (self.is_minus_btn_latch and not minus_button)\n    )\n\n    if self.is_cruise_latch:\n''',
"CarState rearm edge")

# Required final markers.
for marker in (
  "# V4.0 DNGA engagement and hybrid-feedback build.",
  "base_session_allowed = (",
  "longitudinal_session_allowed = (",
  "gas_override_active = (",
  "v40_rearm_edge = (",
  "self.v33r4_overlap_entry_frame = frame",
  "frame - self.v33r4_overlap_entry_frame",
  "longitudinal_enabled = longitudinal_session_allowed and self.CP.openpilotLongitudinalControl",
):
  if marker not in cc:
    raise SystemExit(f"missing controller marker: {marker}")

for marker in ("self.v40_acc_rearm_edge = False", "self.v40_acc_rearm_edge = bool("):
  if marker not in cs:
    raise SystemExit(f"missing CarState marker: {marker}")

CC_PATH.write_text(cc)
CS_PATH.write_text(cs)
py_compile.compile(str(CC_PATH), doraise=True)
py_compile.compile(str(CS_PATH), doraise=True)
print("DNGA V4.0 Sunnypilot adaptation applied; py_compile passed")
