"""Small reusable Matplotlib controls for interactive simulation examples."""

from dataclasses import dataclass, field


@dataclass
class SimulationControls:
    """Mutable button state read by a simulation loop between physics ticks."""

    running: bool = False
    reset_requested: bool = False
    exit_requested: bool = False
    widgets: tuple[object, ...] = field(default_factory=tuple)

    def start(self) -> None:
        """Allow the simulation loop to advance physics."""
        self.running = True

    def stop(self) -> None:
        """Pause the simulation loop without resetting its current state."""
        self.running = False

    def request_reset(self) -> None:
        """Pause the loop and ask its owner to restore its initial state."""
        self.running = False
        self.reset_requested = True

    def consume_reset(self) -> bool:
        """Return and clear one pending reset request from the button panel."""
        requested, self.reset_requested = self.reset_requested, False
        return requested

    def request_exit(self) -> None:
        """Ask the simulation loop to close after its current GUI update."""
        self.exit_requested = True


def add_simulation_buttons(figure: object, y: float = 0.67) -> SimulationControls:
    """Add Start and Restart buttons to an existing Matplotlib figure."""
    from matplotlib.widgets import Button

    controls = SimulationControls()
    start = Button(figure.add_axes((0.35, y, 0.12, 0.025)), "Start")
    reset = Button(figure.add_axes((0.53, y, 0.12, 0.025)), "Restart")
    start.on_clicked(lambda _: controls.start())
    reset.on_clicked(lambda _: controls.request_reset())
    figure.canvas.mpl_connect("close_event", lambda _: controls.request_exit())
    controls.widgets = (start, reset)
    return controls
