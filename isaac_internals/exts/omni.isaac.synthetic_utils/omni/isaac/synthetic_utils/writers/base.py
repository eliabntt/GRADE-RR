# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


"""Base class for writing groundtruth data offline.
"""

import atexit
import queue
import threading


class BaseWriter:
    def __init__(self, data_dir, num_worker_threads, max_queue_size=500):
        atexit.register(self.stop_threads)
        # Threading for multiple scenes
        self.num_worker_threads = num_worker_threads
        # Initialize queue with a specified size
        self.q = queue.Queue(max_queue_size)
        self.data_dir = data_dir
        self.threads = []

    def start_threads(self):
        """Start worker threads."""
        for _ in range(self.num_worker_threads):
            t = threading.Thread(target=self.worker, daemon=True)
            t.start()
            self.threads.append(t)

    def stop_threads(self):
        """Waits for all tasks to be completed before stopping worker threads."""
        print(f"Finish writing data...")

        # Block until all tasks are done
        self.q.join()

        print(f"Done.")

    def worker(self):
        """Processes task from queue. Each tasks contains groundtruth data and metadata which is used to transform the output and write it to disk."""
        pass
