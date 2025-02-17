from textual.app import App, ComposeResult
from textual.widgets import Button


class ButtonApp(App):
    CSS = """
    # Button {
    #     margin-bottom: 1;
    # }

    #none {
    border: none;
    }

    #link {
    border: none;
    height: 1;
    min-width: 0;
    }
    """

    def compose(self) -> ComposeResult:
        yield Button("Default Button")
        yield Button("Border none", id="none")
        yield Button("What I wanted", id="link")
        yield Button("Start", id="start", variant="success")


if __name__ == "__main__":
    app = ButtonApp()
    app.run()