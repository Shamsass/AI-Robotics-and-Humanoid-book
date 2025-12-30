
# import os
# from typing import List, Dict, Any, Optional
# from retry import retry
# import vertexai
# from vertexai.generative_models import GenerativeModel, GenerationConfig

# # Placeholder constants, please replace with your actual values
# GEMINI_AVAILABLE_REGIONS = ["us-central1", "us-east1", "us-west1"]
# GEMINI_URL = "https://{region}-aiplatform.googleapis.com/v1/projects/{project}/locations/{region}/publishers/google/models/{model}:streamGenerateContent"
# GCP_PROJECT = os.environ.get("GCP_PROJECT")
# SAFETY_FILTER_CONFIG = {} # Placeholder for safety filter configuration

# # Placeholder for a caching utility
# def caching(cache):
#     def decorator(func):
#         def wrapper(*args, **kwargs):
#             return func(*args, **kwargs)
#         return wrapper
#     return decorator

# class GeminiModel:
#     """A wrapper class for the Gemini API."""

#     def __init__(self, model_name: str, project: str, location: str):
#         """Initializes the Gemini model."""
#         self.model_name = model_name
#         self.project = project
#         self.location = location
#         vertexai.init(project=self.project, location=self.location)
#         self.model = GenerativeModel(self.model_name)

#     @retry(tries=3, delay=2)
#     @caching(cache={})
#     def _generate_content(self,
#                           prompt: str,
#                           temperature: float = 0.2,
#                           max_output_tokens: int = 2048,
#                           top_p: float = 0.95,
#                           top_k: int = 40,
#                           **kwargs) -> Optional[str]:
#         """Generates content using the Gemini model."""
#         try:
#             generation_config = GenerationConfig(
#                 temperature=temperature,
#                 top_p=top_p,
#                 top_k=top_k,
#                 max_output_tokens=max_output_tokens,
#             )
#             response = self.model.generate_content(
#                 prompt,
#                 generation_config=generation_config,
#                 safety_settings=SAFETY_FILTER_CONFIG,
#                 **kwargs
#             )
#             return response.text
#         except Exception as e:
#             print(f"Error generating content: {e}")
#             return None

#     def generate_text(self,
#                       prompt: str,
#                       temperature: float = 0.2,
#                       max_output_tokens: int = 2048,
#                       **kwargs) -> Optional[str]:
#         """Generates text from a prompt."""
#         return self._generate_content(
#             prompt,
#             temperature=temperature,
#             max_output_tokens=max_output_tokens,
#             **kwargs
#         )
import os
import requests

PROJECT = os.environ["GCP_PROJECT"]
REGION = "us-central1"
MODEL = "gemini-1.5-pro"
ACCESS_TOKEN = os.environ["GCP_ACCESS_TOKEN"]

def generate_text(prompt: str):
    url = f"https://{REGION}-aiplatform.googleapis.com/v1/projects/{PROJECT}/locations/{REGION}/publishers/google/models/{MODEL}:generateContent"

    headers = {
        "Authorization": f"Bearer {ACCESS_TOKEN}",
        "Content-Type": "application/json",
    }

    data = {
        "contents": [{"role": "user", "parts": [{"text": prompt}]}]
    }

    r = requests.post(url, headers=headers, json=data)
    r.raise_for_status()
    return r.json()
