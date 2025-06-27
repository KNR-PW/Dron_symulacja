from langchain_community.chat_models import ChatOpenAI
from langchain_core.messages import HumanMessage
import base64
import requests
import time
import threading

class VisionAgent:
    def __init__(self, message_pool=None):
        self.message_pool = message_pool
        self.llm = ChatOpenAI(
            model="gpt-4-turbo",
            max_tokens=500,
        )

    def describe_image(self, image_path: str):
        with open(image_path, "rb") as f:
            image_base64 = base64.b64encode(f.read()).decode("utf-8")
        image_data_url = f"data:image/jpeg;base64,{image_base64}"
        llm = ChatOpenAI(
            model="gpt-4-turbo",
            max_tokens=500,
        )
        message = HumanMessage(
            content=[
                {"type": "text", "text": """
You are a vision-enabled assistant whose sole task is to analyze a single image and return a JSON object, no additional text. The JSON must have exactly two keys:

  • “contains_orange_light” (boolean) – true if an orange light is visible anywhere in the image, false otherwise.  
  • “light_status” (string) – if “contains_orange_light” is true, this must be either “on” or “off” depending on whether that orange light appears illuminated; if you cannot decide, use “unknown”.

When you receive the image (as raw bytes or base64), you must inspect it for any orange light and decide if it is illuminated. Then reply ONLY with valid JSON matching this schema. Do not include any extra fields, commentary, or markdown—only the JSON object exactly as specified.
                """},
                {"type": "image_url", "image_url": {"url": image_data_url}},
            ]
        )
        response = llm.invoke([message])
        return response.content

# if __name__ == "__main__":
#     agent = VisionAgent()
#     image_path = "/home/stas/Downloads/wetransfer_kjhg_2025-06-03_1851/fotki_dji/DJI_2037.JPG"  # Replace with your image path
#     result = agent.describe_image(image_path)
#     print(result)  # This will print the JSON response from the LLM