"""DNGA V4.3.5 brake-to-propulsion handoff fix.

Keep the staged 0x271 release-pump phase atomic with 0x273 decel framing.
This wrapper intentionally leaves V4.3.4 braking thresholds/timers unchanged
while preventing positive propulsion from being armed before the release pump
has ended and passive HEV feedback has subsequently confirmed a clean handoff.
"""

from opendbc.car.dnga.longitudinal_v50_legacy import LongitudinalController as BaseLongitudinalController
from opendbc.car.dnga.values import LongitudinalParams as P


class LongitudinalController(BaseLongitudinalController):
  def _update_handoff_feedback(self, session, propulsion, brake):
    """Gate propulsion until every commanded decel/release phase is finished."""
    release_active = brake.hydraulic or brake.sng_release or brake.release_pump
    decel_commanded = (
      release_active
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

    # Do not consume the HEV-ready debounce while the 0x271 release pump is
    # still commanded. The ready dwell starts only after the pump phase ends.
    ready_candidate = (
      self.handoff_pending
      and (not release_active)
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

  def _update_speed_offset(self, CS, frame, session, propulsion, plan, brake):
    # A READY/-0.4 release-pump command is still part of brake release. Keep
    # desired speed neutral and do not let positive 0x273 offset build against it.
    if brake.release_pump:
      self.speed_offset = 0.0
      self.handoff_pending = True
      self.handoff_ready_counter = 0
      return
    return super()._update_speed_offset(CS, frame, session, propulsion, plan, brake)

  def _build_command(self, CS, frame, session, plan, lead_state, brake, propulsion):
    command = super()._build_command(CS, frame, session, plan, lead_state, brake, propulsion)

    # Defense in depth: release-pump frames must never carry IS_ACCEL even if
    # another handoff-state change later clears handoff_pending prematurely.
    if session.enabled and brake.release_pump:
      command.speed = CS.out.vEgo
      command.is_accel = False
      command.is_decel = True

    return command
