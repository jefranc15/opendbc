"""DNGA V4.3.6 openpilot-owned longitudinal policy.

Extends the V4.3.5 atomic brake-to-propulsion handoff while removing normal
stock ACC 0x271/0x273 requests from live longitudinal authority. Stock ACC
signals remain decoded in CarState for logging/comparison only. Planner/radar
geometry owns normal brake entry and magnitude; physical HEV feedback remains
the brake-to-propulsion safety gate.
"""

from opendbc.car.dnga.longitudinal_v50_legacy import StopGuard, relative_stop_request
from opendbc.car.dnga.longitudinal_atomic_release import LongitudinalController as AtomicReleaseLongitudinalController
from opendbc.car.dnga.values import LongitudinalParams as P


class LongitudinalController(AtomicReleaseLongitudinalController):
  def _update_stop_guard(self, CS, frame, planner_brake, moving_allowed, lead_state):
    """Use OP planner/radar geometry only; stock ACC is observation-only."""
    # Explicitly neutralize any inherited stock guard state. CarState continues
    # decoding the OEM 0x271/0x273 pair, but those values cannot activate,
    # size, floor, cap, or prolong OP hydraulic braking in this branch.
    self.stock_brake_guard_active = False
    self.stock_brake_release_counter = 0

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

    # Unlike the blocked stock ACC command pair, radar/model geometry is only
    # admitted after matching negative planner intent and the existing debounce.
    predictive_stop_entry = predictive_stop_context and planner_brake >= P.PREDICTIVE_MIN_PLANNER_BRAKE
    if predictive_stop_entry:
      self.predictive_entry_counter = min(self.predictive_entry_counter + 1, P.PREDICTIVE_ENTRY_COUNT)
    else:
      self.predictive_entry_counter = 0
    predictive_stop_confirmed = self.predictive_entry_counter >= P.PREDICTIVE_ENTRY_COUNT

    relative_brake_request = (
      relative_stop_request(CS.out.vEgo, lead_state.distance, lead_state.closing_speed)
      if predictive_stop_context
      else 0.0
    )
    stop_completion_guard = (
      self.stop_guard_latched
      and lead_state.relevant
      and lead_state.status
      and (not CS.out.standstill)
      and (CS.out.vEgo <= P.SNG_ARM_SPEED)
      and (0.0 < lead_state.distance <= P.STOP_COMPLETION_MAX_DISTANCE)
      and (lead_state.speed <= P.STOP_COMPLETION_MAX_LEAD_SPEED)
    )
    stop_guard_authority = predictive_stop_context or stop_completion_guard

    return StopGuard(
      0.0,
      False,
      predictive_stop_confirmed,
      predictive_stop_confirmed,
      relative_brake_request,
      stop_completion_guard,
      stop_guard_authority,
    )
