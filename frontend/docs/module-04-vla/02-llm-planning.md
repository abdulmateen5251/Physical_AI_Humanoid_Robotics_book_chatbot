# Chapter 2: LLM-Based Planning and Execution

## Converting Commands to Plans

An LLM planner converts natural language into executable action sequences:

```python
import openai
from typing import List, Dict
import json

class RobotPlanner:
    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)
        
        # Define allowed actions
        self.allowed_actions = [
            'move_forward', 'turn_left', 'turn_right',
            'grasp', 'release', 'move_arm'
        ]
    
    def plan(self, command: str) -> List[Dict]:
        """Convert natural language command to action plan"""
        
        system_prompt = f"""You are a robot motion planner. Your task is to convert
natural language commands into a sequence of robot actions.

Available actions:
{json.dumps(self.allowed_actions, indent=2)}

Return a JSON array of actions with parameters.

Example:
Input: "Pick up the red block on the table"
Output: [
  {{"action": "move_forward", "distance": 1.0}},
  {{"action": "grasp", "force": 10.0}},
  {{"action": "move_arm", "x": 0.5, "y": 0.5, "z": 1.0}}
]

Be concise and return only valid JSON."""
        
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.3  # Lower temperature for deterministic output
        )
        
        plan_text = response.choices[0].message.content
        
        # Parse JSON from response
        import re
        
        json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
        if json_match:
            plan = json.loads(json_match.group())
            return plan
        else:
            return []

# Example
if __name__ == '__main__':
    planner = RobotPlanner(api_key='your-api-key')
    
    command = "Move forward one meter and then turn left"
    plan = planner.plan(command)
    
    print("Generated plan:")
    for action in plan:
        print(f"  - {action}")
```

## Selection-Mode Constraint Enforcement

When operating in "selection mode" (limited context), enforce that actions only reference information in the selected text:

```python
class ConstrainedPlanner(RobotPlanner):
    def plan_with_selection(self, command: str, selected_text: str) -> List[Dict]:
        """Plan using only information in selected text"""
        
        system_prompt = f"""You are a constrained robot planner. IMPORTANT:
You may ONLY use information present in the selected text below.
Do not use any external knowledge or assumptions.

SELECTED TEXT:
{selected_text}

Natural language command:
{command}

If the selected text does not contain enough information to plan, 
respond with: {{"error": "insufficient_information"}}

Otherwise, return a JSON array of actions."""
        
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.1
        )
        
        return self._parse_response(response)

# Test selection-mode enforcement
if __name__ == '__main__':
    planner = ConstrainedPlanner(api_key='your-api-key')
    
    selected_text = """The robot can move forward or backward at 1 m/s.
The robot has a gripper that can open and close."""
    
    command = "Turn the robot 90 degrees"
    plan = planner.plan_with_selection(command, selected_text)
    
    # Should return error because turning is not mentioned
    print(f"Plan result: {plan}")
```

## Multi-Model Reasoning

**Combine multiple LLMs for robustness:**

```python
class MultiModelPlanner:
    def __init__(self):
        import openai
        self.client = openai.OpenAI()
    
    def plan_with_consensus(self, command: str, num_models: int = 3) -> Dict:
        """Get plan from multiple models and use consensus"""
        
        models = ["gpt-4", "gpt-4-turbo", "gpt-3.5-turbo"][:num_models]
        plans = []
        
        for model in models:
            response = self.client.chat.completions.create(
                model=model,
                messages=[
                    {"role": "user", "content": f"Plan this robot task: {command}"}
                ]
            )
            plans.append(response.choices[0].message.content)
        
        # Analyze consensus
        consensus_result = {
            "plans": plans,
            "agreement_score": self.compute_agreement(plans),
            "recommended_plan": plans[0]  # Use first as recommendation
        }
        
        return consensus_result
    
    def compute_agreement(self, plans: List[str]) -> float:
        """Compute how much plans agree"""
        if len(plans) < 2:
            return 1.0
        
        # Simple agreement metric: check common actions
        import json
        import re
        
        action_sets = []
        for plan in plans:
            json_match = re.search(r'\[.*\]', plan, re.DOTALL)
            if json_match:
                try:
                    actions = json.loads(json_match.group())
                    action_set = set(a.get('action') for a in actions)
                    action_sets.append(action_set)
                except:
                    pass
        
        if not action_sets:
            return 0.0
        
        # Intersection over union
        intersection = set.intersection(*action_sets)
        union = set.union(*action_sets)
        
        return len(intersection) / max(len(union), 1)
```

## Few-Shot Prompting

**Improve accuracy with examples:**

```python
class FewShotPlanner(RobotPlanner):
    def plan_with_examples(self, command: str) -> List[Dict]:
        """Plan with few-shot examples in prompt"""
        
        examples = """Example 1:
Command: "Move forward 2 meters"
Action: [{"action": "move_forward", "distance": 2.0}]

Example 2:
Command: "Pick up the object and move it to the left"
Actions: [
  {"action": "move_forward", "distance": 0.5},
  {"action": "grasp", "force": 15.0},
  {"action": "turn_left"},
  {"action": "move_forward", "distance": 0.5},
  {"action": "release"}
]

Example 3:
Command: "Rotate 180 degrees"
Actions: [
  {"action": "turn_right"},
  {"action": "turn_right"}
]"""
        
        system_prompt = f"""You are a robot motion planner. Convert natural language commands to JSON action sequences.

{examples}

Now plan the following command using the same format."""
        
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.2
        )
        
        return self._parse_response(response)
```

## Key Concepts

- **Temperature**: Lower (0.1-0.3) for deterministic planning
- **Few-shot**: Examples significantly improve accuracy
- **Constraint enforcement**: Use system prompts for safety
- **Consensus**: Multiple models reduce hallucinations
- **Validation**: Always validate generated plans before execution
