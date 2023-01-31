# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

""" This plugin is used to load shapenet objects into Kit.

        If the shape already exists as a USD on a connected omniverse server, then
    it will use that version, unless there is an override.
        If not on omniverse, the plugin will convert the obj from a folder on the
    machine--fetching to the local machine from the web if needed--and upoad it to
    omniverse if there is a connection.
"""
import omni.ext
import omni.kit

from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import threading
from time import sleep, time

from .comm import process_request_in_thread
from .globals import DEBUG_PRINT_ON, g_bind_address
from .menu import ShapenetMenu

from queue import Queue

EXTENSION_NAME = "ShapeNet Loader"

# The listener thread will fill these so that main thread can consume them.
g_requests = Queue()
# The The listener, will respond
g_responses = Queue()


class ShapeNetRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Set-Cookie", "foo=bar")
        self.end_headers()
        self.wfile.write(
            json.dumps(
                {
                    "success": False,
                    "received": "ok",
                    "message": "This Is a Long Drive for Someone with Nothing to Think About",
                }
            )
        )

    def do_POST(self):
        # This recieves the outside posted data, which makes a request to the system.
        request_start = time()
        length = int(self.headers["Content-Length"])
        request = json.loads(self.rfile.read(length))
        g_requests.put(request)
        # wait for the response, and send when you get it.
        while g_responses.empty():
            # print("waiting for response...")
            sleep(0.1)
        if DEBUG_PRINT_ON:
            print("AFTER THE POSSIBLE EXCEP")
        response = g_responses.get()
        response["time"] = time() - request_start

        self.send_response_only(200)
        self.send_header("Set-Cookie", "foo=bar")
        self.end_headers()
        self.wfile.write(json.dumps(response).encode(encoding="utf_8"))

    do_PUT = do_POST
    do_DELETE = do_GET


def run_server(httpd):
    if httpd is not None:
        omni.kit.app.get_app().print_and_log(
            f"omni.isaac.shapenet's receiver for external messages has started. on {g_bind_address}"
        )
        httpd.serve_forever()


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        if DEBUG_PRINT_ON:
            print("\nI STARTED I STARTED!\n")
        self._menu = ShapenetMenu()

        if DEBUG_PRINT_ON:
            print("\nafter ShapenetMenu\n")
        # Creat the TCPServer and run it in another thread, but keep handle to it so
        # we can call shutdown loater.
        server_address = (g_bind_address[0], int(g_bind_address[1]))
        try:
            self._http_server = HTTPServer(server_address, ShapeNetRequestHandler)
        except OSError as e:
            self._http_server = None
            print("HTTPServer failed to start:", e)

        if DEBUG_PRINT_ON:
            print("\nafter http_server\n")
        self.thread = threading.Thread(target=run_server, args=(self._http_server,))

        if DEBUG_PRINT_ON:
            print("\nafter threading\n")
        self.thread.start()

        if DEBUG_PRINT_ON:
            print("\nafter threading.start\n")
        self.update_events = (
            omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)
        )

        if DEBUG_PRINT_ON:
            print("\nafter update_events\n")

    def on_shutdown(self):
        if self._http_server is not None:
            self._http_server.shutdown()
        self.thread.join()

        self._menu.shutdown()
        self._menu = None

    def _on_update(self, dt):
        global g_requests
        if not g_requests.empty():
            request = g_requests.get()
            if DEBUG_PRINT_ON:
                print("Call process_request_in_thread with request: ", request)
            process_request_in_thread("new", g_responses, self._menu, request)

    def get_name(self):
        """Return the name of the extension"""
        return EXTENSION_NAME
