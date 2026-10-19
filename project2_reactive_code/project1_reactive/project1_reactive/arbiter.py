"""
Priority-based arbitration (pseudo-subsumption, after Jones et al.).

Behaviors are checked from highest to lowest priority. The first one that
asks for control wins; everything below it is suppressed for this cycle.
"""

from project1_reactive.behaviors import STOP


class PriorityArbiter:
    """Pick one behavior's command per control cycle by fixed priority."""

    def __init__(self, behaviors):
        """``behaviors``: list ordered from highest to lowest priority."""
        self.behaviors = list(behaviors)
        self.winner = None

    def step(self, state):
        """
        Run one control cycle.

        Returns (winner_name, command). If no behavior wants control the
        robot is stopped and the winner is ``'none'``.
        """
        for index, behavior in enumerate(self.behaviors):
            command = behavior.update(state)
            if command is not None:
                for lower in self.behaviors[index + 1:]:
                    lower.suppress()
                self.winner = behavior.name
                return behavior.name, command
        self.winner = 'none'
        return 'none', STOP
