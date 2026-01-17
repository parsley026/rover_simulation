import argparse

def main() -> int:
    args = argparse.ArgumentParser()
    args.add_argument("--model-file")
    args.add_argument("--pose")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())