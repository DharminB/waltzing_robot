#! /usr/bin/env python

from __future__ import print_function

import tkinter as tk
from pygame import mixer

BGCOLOR='#edead9'
FGCOLOR='#000000'
AUDIO_FILE='/home/dharmin/Music/sample.wav'

class MainApplication(object):
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Waltzing robot')
        self.root.resizable(width=False, height=False)
        self.root.configure(bg=BGCOLOR)
        self.root.configure(height=600)
        self.root.configure(width=800)
        self.status = tk.Label(self.root, text="x: 0, y: 0", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
        self.status.configure(bg=BGCOLOR, fg=FGCOLOR)
        self.canvas = tk.Canvas(self.root, width=600, height=500)
        self.canvas.pack(side=tk.LEFT)
        self.canvas.configure(bg='white')
        self.canvas.bind('<Motion>', self.canvas_motion_cb)
        self.canvas.bind('<Button-1>', self.canvas_click_cb)
        self.canvas.bind('<ButtonRelease-1>', self.canvas_click_cb)
        self.test_button = tk.Button(self.root, text='test', command=self.test_button_cb)
        self.test_button.configure(bg=BGCOLOR, fg=FGCOLOR)
        self.test_button.pack(side=tk.RIGHT)
        self.music_playing = False
        mixer.init()
        self.sound_obj = mixer.Sound(AUDIO_FILE)

    def canvas_motion_cb(self, event):
        self.status.configure(text='x: '+str(event.x)+', y: '+str(event.y))

    def canvas_click_cb(self, event):
        print(event)
        print(event.type)

    def test_button_cb(self):
        print("button pressed")
        if self.music_playing:
            self.sound_obj.stop()
        else:
            self.sound_obj.play()
        self.music_playing = not self.music_playing

if __name__ == "__main__":
    # root = tk.Tk()
    app = MainApplication()#.pack(side="top", fill="both", expand=True)
    app.root.mainloop()
