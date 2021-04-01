#!/usr/bin/env python

from threading import Timer


class RepeatedTimer(object):
    """
    This is a threaded timer to execute a piece of code periodically 
    without blocking the main execution.

    This non-blocking method allows the code to keep running while also
    executing our function every n seconds

    NOTE:
    - start() and stop() are safe to call multiple times even if the timer 
    has already started/stopped

    - You can change interval anytime, it will be effective after next run. 
    Same for args, kwargs and even function!
    """

    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
