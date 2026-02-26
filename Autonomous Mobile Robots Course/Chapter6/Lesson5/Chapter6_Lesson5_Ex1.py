#!/usr/bin/env python3
# Chapter6_Lesson5_Ex1.py
# Exercise: verify normalization and compare FFT convolution to direct O(N^2) prediction

import math
import numpy as np

def predict_direct(bel_prev, kernel):
    N = len(bel_prev)
    bel_bar = np.zeros(N, dtype=float)
    for j in range(N):
        s = 0.0
        for i in range(N):
            # kernel offset (j - i) mod N
            k = (j - i) % N
            s += kernel[k] * bel_prev[i]
        bel_bar[j] = s
    bel_bar = np.maximum(bel_bar, 0.0)
    bel_bar /= np.sum(bel_bar)
    return bel_bar

def predict_fft(bel_prev, kernel):
    B = np.fft.fft(bel_prev)
    K = np.fft.fft(kernel)
    bel_bar = np.real(np.fft.ifft(B * K))
    bel_bar = np.maximum(bel_bar, 0.0)
    bel_bar /= np.sum(bel_bar)
    return bel_bar

def main():
    rng = np.random.default_rng(0)
    N = 128
    bel = rng.random(N)
    bel /= np.sum(bel)

    # random cyclic kernel
    k = rng.random(N)
    k /= np.sum(k)

    d = predict_direct(bel, k)
    f = predict_fft(bel, k)

    print("sum(d) =", float(np.sum(d)))
    print("sum(f) =", float(np.sum(f)))
    print("max|d-f| =", float(np.max(np.abs(d - f))))

if __name__ == "__main__":
    main()
