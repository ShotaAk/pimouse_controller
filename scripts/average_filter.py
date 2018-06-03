#!/usr/bin/env python
#encoding: utf8

import numpy as np
import math

# 移動平均フィルタ
# 平均値を計算するための個数は引数sizeで指定する
class AverageFilter():
    def __init__(self, size):
        if size <= 0:
            size = 1
        self._buffer = np.zeros(size, dtype=np.float32)
        self._buffer_size = size
        self._current_index = 0
        self._filtered_value = 0
        self._offset_value = 0

    def update(self, value):
        self._buffer[self._current_index] = value

        self._current_index += 1
        if self._current_index >= self._buffer_size:
            self._current_index = 0

        self._filtered_value = np.average(self._buffer)

    def get_value(self):
        return self._filtered_value - self._offset_value

    def offset(self):
        self._offset_value = self._filtered_value
