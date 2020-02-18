#! /usr/bin/env python

from __future__ import print_function

import os
import time
import tkinter as tk
import tkSimpleDialog as simpledialog
import tkFileDialog as filedialog
import tkMessageBox as messagebox


class MainApplication(object):

    bgcolor = '#edead9'
    fgcolor = '#000000'

    def __init__(self, connection=None):
        self._gui_init()
        self.connection = connection

    def _gui_init(self):
        self.root = tk.Tk()
        self.root.title('Waltzing robot')
        self.root.configure(bg=self.bgcolor)
        self.root.configure(height=600)
        self.root.configure(width=800)
        self.root.resizable(width=False, height=False)

        self.gui_status = tk.Label(self.root,
                                   text="Welcome to waltzing robot",
                                   bd=1,
                                   relief=tk.SUNKEN,
                                   anchor=tk.W)
        self.gui_status.pack(side=tk.BOTTOM, fill=tk.X)
        self.gui_status.configure(bg=self.bgcolor, fg=self.fgcolor)

        self.add_cp_button = tk.Button(self.root,
                                       text='Add control point',
                                       command=self.add_cp_cb,
                                       state=tk.DISABLED)
        self.add_cp_button.configure(bg=self.bgcolor, fg=self.fgcolor)
        self.add_cp_button.pack(side=tk.TOP)

        self.save_button = tk.Button(self.root,
                                     text='save',
                                     command=self.save_button_cb,
                                     state=tk.DISABLED)
        self.save_button.configure(bg=self.bgcolor, fg=self.fgcolor)
        self.save_button.pack(side=tk.TOP)

        self.load_button = tk.Button(self.root, text='load', command=self.load_button_cb)
        self.load_button.configure(bg=self.bgcolor, fg=self.fgcolor)
        self.load_button.pack(side=tk.TOP)

    def save_button_cb(self):
        print("save button pressed")
        current_dir = os.path.abspath(os.path.dirname(__file__))
        src_dir = os.path.dirname(current_dir)
        ros_dir = os.path.dirname(src_dir)
        waypoints_dir = os.path.join(ros_dir, 'config/waypoints')
        filename = filedialog.asksaveasfilename(title='Save as',
                                                initialdir=waypoints_dir,
                                                defaultextension='.yaml')
        print(filename)
        self.connection.send('save:'+filename)
        self.gui_status.configure(text='Successfully saved')

    def load_button_cb(self):
        print("load button pressed")
        current_dir = os.path.abspath(os.path.dirname(__file__))
        src_dir = os.path.dirname(current_dir)
        ros_dir = os.path.dirname(src_dir)
        waypoints_dir = os.path.join(ros_dir, 'config/waypoints')
        filename = filedialog.askopenfilename(title='Open',
                                              initialdir=waypoints_dir,
                                              filetypes=[('Waypoint config', ('*.yaml'))])
        print(filename)
        self.connection.send('load:'+filename)
        self.gui_status.configure(text='Successfully loaded')
        self.add_cp_button.configure(state=tk.NORMAL)
        self.save_button.configure(state=tk.NORMAL)

    def add_cp_cb(self):
        """Callback for adding a control point point. Ignores the callback if 
        there is no end waypoint. Otherwise, adds a interactive marker at the 
        mid-point of 2 waypoints.

        :returns: None

        """
        print("add_cp_button pressed")
        self.connection.send('add_cp')
        self.gui_status.configure(text='Control point added')

    def mainloop(self):
        while True:
            try:
                gui_exists = 1 == self.root.winfo_exists()
            except Exception as e:
                self.connection.send('close')
                break
            self.root.update()
            time.sleep(0.01)
            if self.connection.poll():
                message = self.connection.recv()
                if message == 'close':
                    break
                if message == 'time':
                    input_time = simpledialog.askfloat("Time", "Time for the newly added waypoint:", parent=self.root)
                    self.connection.send(input_time)
                if message == 'null_time':
                    messagebox.showerror(title='Error', message='Invalid time entered. Please input again')
                    input_time = simpledialog.askfloat("Time", "Time for the newly added waypoint:", parent=self.root)
                    self.connection.send(input_time)
                if message == 'invalid_time':
                    messagebox.showerror(title='Error',
                            message='Time entered was less than previous waypoint\'s time. Please input again')
                    input_time = simpledialog.askfloat("Time", "Time for the newly added waypoint:", parent=self.root)
                    self.connection.send(input_time)
                if 'status:' in message:
                    new_status = message.split(':')[1]
                    self.gui_status.configure(text=new_status)
                if 'enable:' in message:
                    button_name = message.split(':')[1]
                    if button_name == 'add_cp_button':
                        self.add_cp_button.configure(state=tk.NORMAL)
                    if button_name == 'save_button':
                        self.save_button.configure(state=tk.NORMAL)
        self.root.quit()

if __name__ == "__main__":
    app = MainApplication()
    app.root.mainloop()
