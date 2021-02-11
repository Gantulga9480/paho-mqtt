from threading import Thread
import csv
import queue
import time


class Worker:

    def __init__(self, id) -> None:
        self.worker = Thread(target=self.work)
        self.run = True
        self.is_started = False
        self.id = id

    def work(self):
        while self.run:
            if not self.is_started:
                print(f'worker {self.id} start')
                self.is_started = True
            else:
                time.sleep(self.id)
                print(f'{self.id} working')
        print(f'worker {self.id} end')


class Control:

    def __init__(self, workers=None) -> None:
        self.workers = workers
        self.run = True

    def controller(self):
        while self.run:
            com = input('Command: ')
            if com == 's':
                for worker in self.workers:
                    worker.worker.start()
            elif com == 'a':
                for worker in self.workers:
                    worker.run = False
            elif com == 'x':
                self.run = False

w1 = Worker(1)
w2 = Worker(2)

c = Control([w1, w2])
c.controller()
