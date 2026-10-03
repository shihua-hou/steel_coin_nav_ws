#!/usr/bin/python3
"""Static file server for web_ui with no-cache headers (avoid stale index.html killing JS)."""
from __future__ import annotations

import os
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = "/home/linaro/steel_coin_nav_ws/web_ui"
PORT = 8080


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, fmt, *args):
        # quieter than default
        if args and str(args[0]).startswith('"GET /api'):
            return
        super().log_message(fmt, *args)


def main():
    os.chdir(ROOT)
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), partial(NoCacheHandler, directory=ROOT))
    print(f"web_http_nocache on :{PORT} root={ROOT}", flush=True)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
