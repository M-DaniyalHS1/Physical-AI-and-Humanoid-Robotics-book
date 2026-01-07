import sys
import os

# Add the backend/src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.getcwd(), 'backend', 'src'))

# Change to the backend/src directory to allow relative imports
original_cwd = os.getcwd()
os.chdir(os.path.join(original_cwd, 'backend', 'src'))

try:
    # Import using relative paths from the src directory
    from services.chat_service import ChatService

    # Test that the chat service can be imported and instantiated
    chat_service = ChatService()
    print("ChatService imported successfully!")

    # Test that the selected-text-only mode implementation exists
    method = getattr(chat_service, 'process_user_message', None)
    if method:
        print("process_user_message method exists!")

        # Check if the docstring or implementation contains references to selected-text-only
        import inspect
        source = inspect.getsource(method)
        if "selected-text-only" in source.lower():
            print("Selected-text-only mode implementation found in process_user_message!")
        else:
            print("Selected-text-only mode implementation NOT found in process_user_message.")
    else:
        print("process_user_message method does NOT exist!")
finally:
    # Change back to original directory
    os.chdir(original_cwd)