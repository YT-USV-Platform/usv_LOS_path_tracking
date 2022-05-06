#!/usr/bin/env python
# -*- coding: utf-8 -*-

from LOSTracker import LOSPathTracking

if __name__ == "__main__":

    tracker = LOSPathTracking()


    for i in range(100):

        tracker.execute()
        tracker.visualize_tracker()

        