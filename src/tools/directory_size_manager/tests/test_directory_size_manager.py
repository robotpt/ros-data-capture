#!/usr/bin/env python

import os
import unittest
import shutil
from fallocate import fallocate
import time

from directory_size_manager import DirectorySizeManager


PKG = 'directory_size_manager'


class TestDirectorySizeManager(unittest.TestCase):

    def setUp(self):
        self.top_dir = 'temp_testing_directory'
        if os.path.exists(self.top_dir):
            shutil.rmtree(self.top_dir)

        self.nested_dir = 'nested'
        self.dirs = os.path.join(self.top_dir, self.nested_dir)
        os.makedirs(self.dirs)

        self.file_names = [
            os.path.join(self.dirs, name) + '.tmp' for name in ['oldest', 'middle', 'youngest']]
        self.file_size = 1024

        for file_name in self.file_names:
            with open(file_name, "w+b") as f:
                fallocate(f, 0, self.file_size)
                time.sleep(0.1)
        self.dsm = DirectorySizeManager(self.top_dir, self.file_size)

    def tearDown(self):
        shutil.rmtree(self.top_dir)

    def test_get_size(self):
        assert self.dsm.directory_size == len(self.file_names) * self.file_size

    def test_run_and_get_oldest_file(self):
        self.dsm.set_max_file_size(self.file_size)

        assert self.dsm.oldest_file == self.file_names[0]
        self.dsm.run_once()

        assert self.dsm.oldest_file == self.file_names[1]
        self.dsm.run_once()

        for _ in range(5):
            assert self.dsm.oldest_file == self.file_names[2]
            self.dsm.run_once()

        self.dsm.set_max_file_size(0)
        for _ in range(5):
            self.dsm.run_once()
            assert self.dsm.oldest_file is None
