#!/usr/bin/env python

import os


class DirectorySizeManager:

    def __init__(
            self,
            directory,
            max_size_in_bytes,
    ):

        if not os.path.isdir(directory):
            raise ValueError("'{}' is not a valid directory".format(directory))
        if max_size_in_bytes < 0:
            raise ValueError("'{}' is not a valid max size for the directory".format(max_size_in_bytes))

        self._dir = directory
        self._max_size_in_bytes = None
        self.set_max_file_size(max_size_in_bytes)

    @property
    def directory_size(self):
        """
        :return: the size in bytes of a directory and all of its sub directories
        """
        total_size = 0
        for dir_path, dir_names, filenames in os.walk(self._dir):
            for f in filenames:
                fp = os.path.join(dir_path, f)
                # skip if it is symbolic link
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)
        return total_size

    @property
    def oldest_file(self):
        try:
            return min(
                (
                    os.path.join(dirname, filename)
                    for dirname, dirnames, filenames in os.walk(self._dir)
                    for filename in filenames
                ), key=lambda fn: os.stat(fn).st_mtime
            )
        except ValueError:
            return None

    def is_too_large(self):
        return self.directory_size > self._max_size_in_bytes

    def run_once(self):
        if self.is_too_large():
            os.remove(self.oldest_file)

    def set_max_file_size(self, value):
        if value < 0:
            raise ValueError("Kilobytes must be non-negative")
        self._max_size_in_bytes = value


if __name__ == '__main__':
    dsm = DirectorySizeManager('/root/upload', 1024)
    print dsm.directory_size
    print dsm.oldest_file
    dsm.run_once()
    print dsm.directory_size
