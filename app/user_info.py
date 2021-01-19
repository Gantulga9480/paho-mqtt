from tkinter import *
from tkinter import ttk


class UserInfo(Tk):

    def __init__(self, master):
        self.age = "User"
        self.sex = "User"
        self.height = "User"
        self.weight = "User"
        self.__options = ["Male", "Female"]
        self.__selected = StringVar()
        self.master = master
        self.win = Toplevel(self.master)
        self.win.attributes("-alpha", 1)
        self.win.resizable(0, 0)

        # Name
        self.age = Label(self.win, text="Age", font=("default", 10))
        self.age.grid(row=0, column=0)
        self.age_entr = ttk.Entry(self.win, font=("default", 10), width=30)
        self.age_entr.grid(row=0, column=1)

        # Sex
        self.age = Label(self.win, text="Age", font=("default", 10))
        self.age.grid(row=1, column=0)
        self.sex_menu = ttk.Combobox(self.win, value=self.__options,
                                     textvariable=self.__selected)
        self.sex_menu.current(0)
        self.sex_menu.config(state="readonly", width=10)
        self.sex_menu.bind("<<ComboboxSelected>>")
        self.sex_menu.grid(row=1, column=1)

        # Height
        self.height = Label(self.win, text="Height", font=("default", 10))
        self.height.grid(row=2, column=0)
        self.height_entr = ttk.Entry(self.win, font=("default", 10), width=30)
        self.height_entr.grid(row=2, column=1)

        self.weight = Label(self.win, text="Weight", font=("default", 10))
        self.weight.grid(row=3, column=0)
        self.weight_entr = ttk.Entry(self.win, font=("default", 10), width=30)
        self.weight_entr.grid(row=3, column=1)

        # Save button
        self.save_btn = ttk.Button(self.win,
                                   text="Enter", command=self.save_info)
        self.save_btn.grid(row=4, column=0, columnspan=2)

    def save_info(self):
        try:
            self.age = int(self.age_entr.get())
            self.sex = self.__selected.get()
            self.height = int(self.height_entr.get())
            self.weight = int(self.weight_entr.get())
            self.win.destroy()
        except:
            pass