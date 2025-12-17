
import argparse

def setup_gemini_config(parser: argparse.ArgumentParser):
    """Adds Gemini-related arguments to the argument parser."""
    gemini_group = parser.add_argument_group("Gemini settings")
    gemini_group.add_argument(
        "--gemini-model",
        type=str,
        default="gemini-1.5-pro-preview-0409",
        help="The Gemini model to use.",
    )
    gemini_group.add_argument(
        "--gcp-project",
        type=str,
        default=None,
        help="The GCP project to use for Gemini.",
    )
    gemini_group.add_argument(
        "--gcp-location",
        type=str,
        default="us-central1",
        help="The GCP location to use for Gemini.",
    )
    gemini_group.add_argument(
        "--gemini-temperature",
        type=float,
        default=0.2,
        help="The temperature to use for Gemini.",
    )
    gemini_group.add_argument(
        "--gemini-max-output-tokens",
        type=int,
        default=2048,
        help="The maximum number of output tokens to use for Gemini.",
    )
    gemini_group.add_argument(
        "--gemini-top-p",
        type=float,
        default=0.95,
        help="The top-p value to use for Gemini.",
    )
    gemini_group.add_argument(
        "--gemini-top-k",
        type=int,
        default=40,
        help="The top-k value to use for Gemini.",
    )
