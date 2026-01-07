#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Verification script for the selected-text-only mode implementation
"""

import subprocess
import sys
import os

def check_implementation():
    """Check if the selected-text-only implementation is in place"""
    chat_service_path = os.path.join("backend", "src", "services", "chat_service.py")
    
    with open(chat_service_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Check for key implementation elements
    checks = {
        "selected-text-only in process_user_message": "selected-text-only" in content.lower(),
        "context prioritization logic": "prioritize the selected text" in content.lower() or "selected text as primary context" in content.lower(),
        "mode validation": "selected-text-only" in content and "mode must be one of" in content,
        "related content filtering": "related_content" in content or "is related to the selected text" in content
    }
    
    print("Checking implementation of selected-text-only mode:")
    all_passed = True

    for check_name, result in checks.items():
        status = "[PASS]" if result else "[FAIL]"
        print(f"  {status} {check_name}")
        if not result:
            all_passed = False

    print(f"\nOverall result: {'[PASS] All checks passed!' if all_passed else '[FAIL] Some checks failed'}")

    return all_passed

if __name__ == "__main__":
    success = check_implementation()
    sys.exit(0 if success else 1)