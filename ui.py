# will likely later be implemented as subclass inheriting from any
# classes requiring ui

# standard library imports

# 3rd part imports
import krpc

# local app imports
from base import *


class TeleUI(Craft):

    def __init__(self, number_of_channels):
        super().__init__()
        self._canvas = self.conn.ui.stock_canvas
        self._screen_size = self._canvas.rect_transform.size
        self._panel = self._canvas.add_panel()
        self._rect = self._panel.rect_transform
        self._rect.size = (125, 10)
        self._rect.position = (110 - self._screen_size[0]/2, 0)
        # (width, height)
        self._channel_count = number_of_channels
        # could eliminate this attribute & parameter, and define it in
        # parents classes. Also add parent.channel_count to _add_text()

    # Create a panel.add_text() variable once for every stream
    # created by the parent class. Define its position in the tele
    # panel and text size and color.
    @property
    def _add_text(self):
        add_text_list = []
        self._rect.size = (125, 10*self._channel_count)
        for i in range(self._channel_count):
            add_text_list.append(self._panel.add_text(str(i)))
        for i in range(len(add_text_list)):
            add_text_list[i].rect.transform.position = (
                0, self._rect.size[1]/2 - 5 - 10*i)
            add_text_list[i].color = (1, 1, 1)
            add_text_list[i].size = 9
        return add_text_list

    def remove_ui(self):
        self._panel.remove()




