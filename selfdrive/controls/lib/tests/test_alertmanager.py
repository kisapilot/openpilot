import random

from openpilot.selfdrive.controls.lib.events import Alert, EVENTS
from openpilot.selfdrive.controls.lib.alertmanager import AlertManager


class TestAlertManager:

  def test_duration(self):
    """
      Enforce that an alert lasts for max(alert duration, duration the alert is added)
    """
    for duration in range(1, 100):
      alert = None
      while not isinstance(alert, Alert):
        event = random.choice([e for e in EVENTS.values() if len(e)])
        alert = random.choice(list(event.values()))

      alert.duration = duration

      # check two cases:
      # - alert is added to AM for <= the alert's duration
      # - alert is added to AM for > alert's duration
      for greater in (True, False):
        if greater:
          add_duration = duration + random.randint(1, 10)
        else:
          add_duration = random.randint(1, duration)
        show_duration = max(duration, add_duration)

        AM = AlertManager()
        for frame in range(duration+10):
          if frame < add_duration:
            AM.add_many(frame, [alert, ])
          current_alert = AM.process_alerts(frame, {})

          shown = current_alert is not None
          should_show = frame <= show_duration
          assert shown == should_show, f"{frame=} {add_duration=} {duration=}"
