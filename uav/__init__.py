"""UAV section: fly a Tello over Wi-Fi from the console and capture its camera footage.

    tello.py   the link to the drone (text commands over UDP, status, stick commands, safety rules)
    plan.py    flight plans: a list of steps the drone flies by itself, saved plans, a copied manual flight
    sim.py     a simulated Tello on this computer (same protocol, synthetic camera) for practice and tests
    h264.py    the minimal H.264 writer the simulated camera uses
    video.py   receives and decodes the drone's video in a separate process: preview, recording, photos

Everything here runs on the edge laptop. Nothing in this package talks to the UGV.
"""
