import asyncio
from typing import Optional
from contextlib import AsyncExitStack

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# from anthropic import Anthropic
from dotenv import load_dotenv
from openai import OpenAI
import os
import time

system_prompt_en = '''
    as a helpful assistant, you will answer user queries and use tools to get information.
    you will use the tools provided by the server to get information.
    if you need to call tools, you will return the function call in the format:
    {
        [FC]:funcname1(para1=argu1);
        [FC]:funcname2(para1=argu1, para2=argu2);
    }
    you will not call the tools directly, but return the function call in the format above.

    the tools you can use are:
'''

program_dir = "C:/Users/14663/AppData/Local/Programs/Python/Python312;C:/Users/14663/AppData/Local/Programs/Python/Python312/Scripts;"
os.environ["PATH"] += os.pathsep + program_dir

# judge whether the message contains a tool call
def judge_tool_call(content):
    content_split = content.split("\n")
    for i in content_split:
        if "[FC]" in i:
            return True
    return False


def tool_calls_format(tool_calls_str: str):
    '''
    {
        [FC]:get_alerts(state=CA);
    }
    to
    [
        {
            "name": "get_alerts",
            "args": {
                "state": "CA"
            }
        }
    ]
    '''
    tool_calls = []
    tools_split = tool_calls_str.split("\n")
    for i in tools_split:
        if "[FC]" in i:
            # 提取函数名称  [Funcall]:map_create();
            funcName = i.split(":")[1].split("(")[0].strip()
            # 提取参数
            args_str = i.split("(")[1].split(")")[0].strip()
            args_dict = {}
            if args_str:
                args_list = args_str.split(",")
                for arg in args_list:
                    key, value = arg.split("=")
                    args_dict[key.strip()] = value.strip().strip("'")
            tool_calls.append({
                "name": funcName,
                "args": args_dict
            })
    return tool_calls


class MCPClient:
    def __init__(self):
        # Initialize session and client objects
        self.session: Optional[ClientSession] = None
        self.exit_stack = AsyncExitStack()
        self.client = OpenAI(
            base_url="https://api.deepseek.com",
            api_key="sk-ca097724a54a4a6e860f97e82a8dd2d5",
        )

    async def connect_to_server(self, server_script_path: str):
        """Connect to an MCP server

        Args:
            server_script_path: Path to the server script (.py)
        """
        server_params = StdioServerParameters(
            command="python3.12",
            args=[server_script_path],
            env=None
        )

        stdio_transport = await self.exit_stack.enter_async_context(stdio_client(server_params))
        self.stdio, self.write = stdio_transport
        self.session = await self.exit_stack.enter_async_context(ClientSession(self.stdio, self.write))

        await self.session.initialize()

        # List available tools
        response = await self.session.list_tools()
        tools = response.tools
        for tool in tools:
            print(f"Tool: {tool.name}, Description: {tool.description}")

    async def process_query(self, query: str) -> str:

        # get available tools from server
        response = await self.session.list_tools()
        available_tools = [{
            "name": tool.name,
            "description": tool.description,
            "input_schema": tool.inputSchema
        } for tool in response.tools]

        # current query tools
        query_prompt = system_prompt_en
        for tool in available_tools:
            print(f"in this query Available tool: {tool['name']} - {tool['description']}")
            query_prompt += f"{tool['name']}: {tool['description']}\n"

        # print(f"debug query_prompt: {query_prompt}\n\n\n")

        """Process a query using Claude and available tools"""
        messages = [
            {
                "role": "system",
                "content": query_prompt
            },
            {
                "role": "user",
                "content": query
            }
        ]

        # Initial Claude API call - 本demo中替换成deepseek
        start_time = time.time()
        response = self.client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            # tools=available_tools
        )
        end_time = time.time()

        content = response.choices[0].message.content
        print("debug response:\n", content, "\ndebug take time:", end_time - start_time)

        # Process response and handle tool calls
        tool_results = []
        final_text = []

        while judge_tool_call(content) == True:

            tool_calls = tool_calls_format(content[content.find("{"):content.rfind("}") + 1])  # str -> list
            print("debug tool_calls:\n", tool_calls)
            for tool in tool_calls:
                tool_name = tool["name"]
                tool_args = tool["args"]

                print(f"debug tool call: {tool_name} with args {tool_args}")

                # eg : result = await self.session.call_tool("get_alerts", {"state": "CA"})
                # eg : result = await self.session.call_tool("get_forecast", {"latitude": 37.7749, "longitude": -122.4194})
                result = await self.session.call_tool(tool_name, tool_args)

                tool_results.append({
                    "call": tool_name,
                    "result": result.content
                })
                final_text.append(f"[Calling tool {tool_name} with args {tool_args}]")

                # add llm response to messages
                messages.append({
                    "role": "assistant",
                    "content": content
                })

                # add tool call result to messages
                messages.append({
                    "role": "user",
                    "content": f"Calling tool {tool_name} with args {tool_args} returned: {result.content}",
                })

                # Get next response from llm
                response = self.client.chat.completions.create(
                    model="deepseek-chat",
                    messages=messages,
                )

                # loop through response content
                content = response.choices[0].message.content

        # out of loop, no more tool calls
        final_text.append(content)

        return "\n".join(final_text)

    async def chat_loop(self):
        """Run an interactive chat loop"""
        print("\nMCP Client Started!")
        print("Type your queries or 'quit' to exit.")

        try:
            print("debug test one query with no input")
            query = "告诉我你现在可以调用哪些函数"
            response = await self.process_query(query)
            print("\n" + response)
        except Exception as e:
            print(f"\nError: {str(e)}")

        return

        while True:
            try:
                query = input("\nQuery: ").strip()

                if query.lower() == 'quit':
                    break
                print("get query:", query)
                response = await self.process_query(query)
                print("\n" + response)

            except Exception as e:
                print(f"\nError: {str(e)}")

    async def cleanup(self):
        """Clean up resources"""
        await self.exit_stack.aclose()


async def main():
    load_dotenv(dotenv_path="./brain/config")  # load environment variables from .env

    # if len(sys.argv) < 2:
    #     print("Usage: python client.py <path_to_server_script>")
    #     sys.exit(1)
    # server_path = sys.argv[1]
    server_path = "D:/code/2025/emabled_ai/DeepEmbody/manager/eaios_decorators.py"

    client = MCPClient()
    try:
        await client.connect_to_server(server_path)
        await client.chat_loop()
    finally:
        await client.cleanup()


if __name__ == "__main__":
    import sys

    asyncio.run(main())

