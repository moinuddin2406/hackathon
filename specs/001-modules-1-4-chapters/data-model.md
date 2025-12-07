# Data Model: Content Structure for Modules 1-4 Chapters

**Feature**: Detailed Chapters for Modules 1-4
**Branch**: `001-modules-1-4-chapters`
**Date**: 2025-12-07

## Purpose

Define the schema, validation rules, and relationships for educational content entities (chapters, diagrams, code examples) to ensure consistency and quality across all four modules.

## Entity Definitions

### Entity 1: Chapter

**Purpose**: Self-contained learning module covering one course module

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `title` | string | Yes | Length 10-100 chars | Chapter title matching module name |
| `module_number` | integer | Yes | Range 1-4 | Sequential module identifier |
| `file_path` | string | Yes | Format: `docs/moduleX/*.md` | Markdown file location |
| `word_count` | integer | Yes | Range 1200-1600 | Content length (excluding code blocks) |
| `mermaid_diagrams` | array[MermaidDiagram] | Yes | Length 4-6 | Visual representations |
| `code_examples` | array[CodeExample] | Yes | Length 5-8 | Runnable Python snippets |
| `navigation_next` | string | Yes | Valid internal link | Link to next chapter |
| `readability_score` | float | Yes | Range 10.0-12.0 | Flesch-Kincaid grade level |
| `learning_objectives` | array[string] | Yes | Length 3-5 | What students will learn |
| `prerequisites` | array[string] | No | - | Required prior knowledge |
| `references` | array[Reference] | Yes | Min 5 items | Official documentation links |

**Validation Rules**:
```
ASSERT word_count >= 1200 AND word_count <= 1600
ASSERT len(mermaid_diagrams) >= 4 AND len(mermaid_diagrams) <= 6
ASSERT len(code_examples) >= 5 AND len(code_examples) <= 8
ASSERT readability_score >= 10.0 AND readability_score <= 12.0
ASSERT len(learning_objectives) >= 3 AND len(learning_objectives) <= 5
ASSERT len(references) >= 5
ASSERT all(ex.free_tier_compatible for ex in code_examples) == True
ASSERT navigation_next starts with "/docs/" or "../"
```

**Relationships**:
- Has-Many: `mermaid_diagrams` (4-6 diagrams per chapter)
- Has-Many: `code_examples` (5-8 examples per chapter)
- Has-Many: `references` (≥5 official doc links per chapter)
- References-One: `navigation_next` (link to next chapter)

**Example**:
```yaml
title: "The Robotic Nervous System (ROS 2)"
module_number: 1
file_path: "docs/module1/robotic-nervous-system-ros2.md"
word_count: 1450
mermaid_diagrams: [...]  # 5 diagrams
code_examples: [...]     # 7 examples
navigation_next: "/docs/module2/digital-twin-gazebo-unity"
readability_score: 11.2
learning_objectives:
  - "Understand ROS 2 architecture and design principles"
  - "Implement basic publisher-subscriber nodes using rclpy"
  - "Explain the role of DDS in ROS 2 communication"
prerequisites:
  - "Intermediate Python programming"
  - "Basic command-line usage"
references: [...]  # 6 official doc links
```

---

### Entity 2: MermaidDiagram

**Purpose**: Visual representation of architecture, workflow, or data flow

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `type` | enum | Yes | One of: flowchart, sequence, class, state | Diagram category |
| `title` | string | Yes | Length 10-80 chars | Diagram purpose description |
| `mermaid_syntax` | text | Yes | Valid Mermaid.js code | Diagram definition |
| `render_validation` | boolean | Yes | Must be true | Confirms successful rendering |
| `max_nodes` | integer | No | Max 12 | Complexity limit (optional enforcement) |
| `description` | string | Yes | Length 20-200 chars | Plain-text explanation (accessibility) |

**Validation Rules**:
```
ASSERT type in ["flowchart", "sequence", "class", "state"]
ASSERT render_validation == true
ASSERT mermaid_syntax.startswith(type) OR mermaid_syntax.startswith("graph")
ASSERT len(title) >= 10 AND len(title) <= 80
ASSERT len(description) >= 20 AND len(description) <= 200
```

**Relationships**:
- Belongs-To: `chapter` (each diagram part of one chapter)

**Example**:
```yaml
type: "flowchart"
title: "ROS 2 Publisher-Subscriber Architecture"
mermaid_syntax: |
  flowchart LR
      Publisher[Publisher Node] -->|/topic| Topic[Topic]
      Topic -->|/topic| Subscriber[Subscriber Node]
      Publisher -.->|DDS Layer| DDS[Data Distribution Service]
      Subscriber -.->|DDS Layer| DDS
render_validation: true
max_nodes: 5
description: "Diagram showing how ROS 2 nodes communicate via topics using the underlying DDS layer for transport."
```

---

### Entity 3: CodeExample

**Purpose**: Runnable Python/rclpy snippet demonstrating concept

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `language` | string | Yes | Must be "python" | Programming language |
| `description` | string | Yes | Length 15-150 chars | What the code demonstrates |
| `code` | text | Yes | Min 50 chars | Executable Python code |
| `dependencies` | array[string] | Yes | Valid pip packages | Required packages (e.g., ["rclpy", "numpy"]) |
| `free_tier_compatible` | boolean | Yes | Must be true | Confirms CPU-only execution |
| `expected_output` | text | No | - | Sample output for validation |
| `execution_time` | string | No | Format "X seconds" | Estimated runtime on CPU |
| `inline_comments` | boolean | Yes | Must be true | Confirms key steps explained |

**Validation Rules**:
```
ASSERT language == "python"
ASSERT free_tier_compatible == true
ASSERT len(code) >= 50
ASSERT len(description) >= 15 AND len(description) <= 150
ASSERT inline_comments == true
ASSERT all(dep in ALLOWED_DEPENDENCIES for dep in dependencies)
ASSERT "import os" not in code OR "API_KEY" not in code  # No secrets
ASSERT "torch.cuda" not in code  # No GPU required
```

**Allowed Dependencies** (Free-Tier Compatible):
```python
ALLOWED_DEPENDENCIES = [
    "rclpy", "numpy", "opencv-python", "openai-whisper",
    "matplotlib", "scipy", "pandas", "scikit-learn",
    "sounddevice", "pyaudio", "pyyaml", "pytest"
]
```

**Relationships**:
- Belongs-To: `chapter` (each code example part of one chapter)

**Example**:
```yaml
language: "python"
description: "Basic ROS 2 publisher node sending string messages"
code: |
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class MinimalPublisher(Node):
      def __init__(self):
          super().__init__('minimal_publisher')
          # Create publisher on 'topic' with queue size 10
          self.publisher_ = self.create_publisher(String, 'topic', 10)
          # Set up timer to publish every 0.5 seconds
          timer_period = 0.5
          self.timer = self.create_timer(timer_period, self.timer_callback)
          self.i = 0

      def timer_callback(self):
          msg = String()
          msg.data = f'Hello World: {self.i}'
          self.publisher_.publish(msg)
          self.get_logger().info(f'Publishing: "{msg.data}"')
          self.i += 1

  def main(args=None):
      rclpy.init(args=args)
      node = MinimalPublisher()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
dependencies: ["rclpy"]
free_tier_compatible: true
expected_output: |
  [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
execution_time: "Continuous (runs until stopped)"
inline_comments: true
```

---

### Entity 4: Reference

**Purpose**: Link to official documentation validating technical claims

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `title` | string | Yes | Length 10-100 chars | Human-readable reference name |
| `url` | string | Yes | Valid HTTPS URL | Official documentation link |
| `source` | enum | Yes | One of: ros2, gazebo, nvidia, openai, other | Documentation provider |
| `last_verified` | date | Yes | ISO 8601 format | When link was last checked |
| `snippet` | string | No | Max 200 chars | Brief excerpt or description |

**Validation Rules**:
```
ASSERT url.startswith("https://")
ASSERT source in ["ros2", "gazebo", "nvidia", "openai", "other"]
ASSERT last_verified <= today()
ASSERT len(title) >= 10 AND len(title) <= 100
```

**Relationships**:
- Belongs-To: `chapter` (each reference supports one chapter)

**Example**:
```yaml
title: "ROS 2 Humble Documentation - Tutorials"
url: "https://docs.ros.org/en/humble/Tutorials.html"
source: "ros2"
last_verified: "2025-12-07"
snippet: "Official ROS 2 Humble tutorials covering basic concepts, client libraries, and advanced features."
```

---

## Content Structure Schema

### Chapter File Structure (Markdown)

```markdown
# Chapter Title

> **Module**: X | **Estimated Reading Time**: Y minutes

## Learning Objectives

- Objective 1
- Objective 2
- Objective 3

## Prerequisites

- Prerequisite 1
- Prerequisite 2

---

## Section 1: Introduction

[2-3 paragraphs introducing the topic, explaining why it matters, and providing context]

## Section 2: Core Concept A

[Detailed explanation with active voice, concrete examples]

### Mermaid Diagram: [Diagram Title]

```mermaid
[Mermaid syntax here]
```

[Plain-text explanation of diagram for accessibility]

### Code Example: [Example Description]

```python
[Python code with inline comments]
```

**What this code does**: [Brief explanation]

**Expected output**:
```
[Sample output]
```

## Section 3: Core Concept B

[Repeat pattern: explanation → diagram → code example]

## Section 4: Practical Application

[Real-world use case demonstrating concepts in action]

## Summary

[2-3 paragraphs recapping key takeaways]

## References

- [Reference 1 Title](https://url1)
- [Reference 2 Title](https://url2)
- ...

## Next Steps

Continue to [Next Chapter Title](../moduleX/next-chapter.md) →
```

---

## Validation Workflow

### Automated Validation (CI/CD)

**Step 1: Word Count & Readability**
```bash
python src/utils/readability-checker.py docs/moduleX/chapter.md
# Expected output: ✅ Word count within range, ✅ Readability grade within range
```

**Step 2: Code Example Execution**
```bash
pytest tests/integration/test_code_examples.py::test_chapter_X
# Expected output: All 5-8 code examples execute without errors
```

**Step 3: Mermaid Diagram Rendering**
```bash
npm run build
# Expected output: Docusaurus build succeeds, all Mermaid diagrams render
```

**Step 4: Link Integrity**
```bash
npm run build # with onBrokenLinks: 'throw'
# Expected output: Zero broken internal or external links
```

### Manual Validation

**Step 5: Content Accuracy Review**
- Compare technical claims against official documentation sources
- Verify all `references` links are functional and relevant
- Confirm free-tier compatibility of all code examples (no GPU, paid APIs)

**Step 6: Constitutional Compliance Audit**
- Simplicity: First-read clarity (peer review)
- Accuracy: Claims traceable to official docs
- Minimalism: No unnecessary content
- Speed: Build time < 2 minutes validated
- Free-Tier: All examples CPU-only confirmed
- Source-of-Truth RAG: Clean heading structure verified

---

## Data Model Summary

| Entity | Attributes | Validation Rules | Relationships |
|--------|------------|------------------|---------------|
| **Chapter** | 11 attributes | 7 validation assertions | Has-Many: diagrams, examples, references |
| **MermaidDiagram** | 6 attributes | 5 validation assertions | Belongs-To: chapter |
| **CodeExample** | 8 attributes | 8 validation assertions | Belongs-To: chapter |
| **Reference** | 5 attributes | 4 validation assertions | Belongs-To: chapter |

**Total Entities**: 4
**Total Validation Rules**: 24 assertions
**Free-Tier Enforcement**: Mandatory at code example level

---

## Next Steps

1. ✅ Data model defined - entity schemas complete
2. → Create contracts/ directory with chapter template and validation rules
3. → Write quickstart.md with contributor setup instructions
4. → Update agent context with data model structure
5. → Proceed to Phase 2 task generation (/sp.tasks command)

---

**Last Updated**: 2025-12-07
