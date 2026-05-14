"""KeyBindings — separates handler registration from documentation rows."""
import pygame as pg


def _key_name(key):
    name = pg.key.name(key)
    return name.upper() if len(name) == 1 else name


class KeyBindings:
    """Holds two independent collections:
    - handlers: (key, callable, repeat) — what actually fires on input.
    - docs: (display, label, repeat) — what the keybindings panel shows.

    Several handlers can share a single doc row (e.g. arrow keys → "move obstacle")
    by registering handlers without a doc and then calling `document(display, label, ...)`
    once for the group.
    """

    def __init__(self):
        self._handlers = []  # (key, handler, repeat)
        self._docs = []      # (display, label, repeat)

    def bind(self, key, handler, repeat=False, doc=None):
        self._handlers.append((key, handler, repeat))
        if doc is not None:
            self._docs.append((_key_name(key), doc, repeat))

    def document(self, display, label, repeat=False):
        self._docs.append((display, label, repeat))

    def dispatch(self, event):
        if event.type != pg.KEYDOWN:
            return
        for key, handler, repeat in self._handlers:
            if not repeat and key == event.key:
                handler()

    def poll(self, pressed):
        for key, handler, repeat in self._handlers:
            if repeat and pressed[key]:
                handler()

    def docs(self):
        return list(self._docs)
