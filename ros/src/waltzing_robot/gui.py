#! /usr/bin/env python

from __future__ import print_function

import rospy
import tkinter as tk
from waltzing_robot.music_player import MusicPlayer

BGCOLOR='#edead9'
FGCOLOR='#000000'
CANVAS_BG='#303030'
CANVAS_GRID='#525252'
AUDIO_FILE='/home/dharmin/Music/waltz_trimmed.wav'

class GuiCanvas(object):

    """Wrapper on Canvas object of TK for this application"""

    def __init__(self, root, width=600, height=500):
        """
        :root: tk.Tk object

        """
        self.scale = 100 # pixel per meter
        self.width = width
        self.height = height
        self.center_x = self.width/2
        self.center_y = self.height/2

        self._root = root
        self.canvas = tk.Canvas(self._root, width=self.width, height=self.height)
        self.canvas.pack(side=tk.LEFT)
        self.canvas.configure(bg=CANVAS_BG)
        self.canvas.bind('<Motion>', self.canvas_motion_cb)
        self.canvas.bind('<Button-1>', self.canvas_click_cb)
        self.canvas.bind('<ButtonRelease-1>', self.canvas_click_cb)

        self.grid_lines = []
        self._draw_grid()

    def canvas_motion_cb(self, event):
        self.status.configure(text='x: '+str(event.x)+', y: '+str(event.y))

    def canvas_click_cb(self, event):
        print(event)
        print(event.type)

    def _draw_grid(self):
        grid_size = 1
        grid_line_id = self.canvas.create_line(0, 0, self.center_x, self.center_y, fill=CANVAS_GRID)
        self.grid_lines.append(grid_line_id)

        

class MainApplication(object):
    def __init__(self):
        self._gui_init()

        self.music_playing = False
        self.music_player = MusicPlayer(AUDIO_FILE)

    def _gui_init(self):
        self.root = tk.Tk()
        self.root.title('Waltzing robot')
        self.root.configure(bg=BGCOLOR)
        self.root.configure(height=600)
        self.root.configure(width=800)
        self.root.resizable(width=False, height=False)
        self.status = tk.Label(self.root, text="x: 0, y: 0", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
        self.status.configure(bg=BGCOLOR, fg=FGCOLOR)
        # self.canvas = GuiCanvas(self.root)
        self.test_button = tk.Button(self.root, text='test', command=self.test_button_cb)
        self.test_button.configure(bg=BGCOLOR, fg=FGCOLOR)
        self.test_button.pack(side=tk.RIGHT)
        # def show_values():
        #     print (w1.get(), w2.get())

        # master = Tk()
        # w1 = Scale(master, from_=0, to=42)
        # w1.pack()
        # w2 = Scale(master, from_=0, to=200, orient=HORIZONTAL)
        # w2.pack()
        # Button(master, text='Show', command=show_values).pack()


    def test_button_cb(self):
        print("button pressed")
        if self.music_playing:
            self.music_player.stop_playing()
        else:
            self.music_player.start_playing()
        self.music_playing = not self.music_playing

if __name__ == "__main__":
    # root = tk.Tk()
    app = MainApplication()#.pack(side="top", fill="both", expand=True)
    app.root.mainloop()
