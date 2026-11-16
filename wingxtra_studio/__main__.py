"""Starts the Wingxtra Studio server:  python -m wingxtra_studio [--port 8000]"""

import argparse

import uvicorn


def main() -> None:
    parser = argparse.ArgumentParser(description="Wingxtra Studio")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    uvicorn.run("wingxtra_studio.api:app", host=args.host, port=args.port)


if __name__ == "__main__":
    main()
