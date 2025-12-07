# Implementation Plan: Detailed Chapters for Modules 1-4

**Branch**: `001-modules-1-4-chapters` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-modules-1-4-chapters/spec.md`

## Summary

Deliver four production-ready educational chapters (Modules 1-4) covering ROS 2, Digital Twin Simulation, NVIDIA Isaac, and Vision-Language-Action systems for the Physical AI & Humanoid Robotics textbook. Each chapter will be 1200-1600 words with 4-6 Mermaid diagrams and 5-8 runnable Python code examples, optimized for free-tier infrastructure and designed for RAG chatbot ingestion. Content must achieve Flesch-Kincaid readability grade 10-12 and build in under 2 minutes on GitHub Actions.

**Technical Approach**: Research-concurrent content generation where chapter writing happens in parallel with technical validation. Use official documentation (ROS 2, Gazebo, NVIDIA Isaac, OpenAI Whisper) as primary sources. Implement MDX format with collapsible code blocks for Docusaurus. Validate build performance and readability continuously.

## Technical Context

**Language/Version**: Markdown/MDX for content; Python 3.8+ for code examples; Node.js 18+ for Docusaurus build
**Primary Dependencies**:
- Docusaurus 3.x (static site generator)
- Mermaid.js (diagram rendering)
- Python rclpy (ROS 2 client library)
- textstat (Flesch-Kincaid readability validation)
**Storage**: Static markdown files in `docs/` directory; no database required
**Testing**:
- Docusaurus build validation (`npm run build`)
- Link checker (onBrokenLinks: 'throw')
- Readability scoring (textstat library)
- Code example execution tests (pytest for Python snippets)
**Target Platform**: GitHub Pages (static hosting); works on all modern browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Static documentation site (single-page application with client-side routing)
**Performance Goals**:
- Build time < 2 minutes on GitHub Actions free tier
- First Contentful Paint < 2s on 3G connection
- Total bundle size < 350 MB including embeddings
**Constraints**:
- Zero broken links (build must pass)
- Free-tier only (no paid APIs, GPU, or services)
- Flesch-Kincaid readability grade 10-12
- All code examples must run CPU-only
**Scale/Scope**:
- 4 chapters × 1200-1600 words = 5000-6400 total words
- 4 chapters × 4-6 diagrams = 16-24 Mermaid diagrams
- 4 chapters × 5-8 code examples = 20-32 Python snippets

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Simplicity ✅
- **Requirement**: Every sentence understandable on first read by intermediate engineering students
- **Plan Alignment**: Flesch-Kincaid grade 10-12 enforced; active voice and concrete examples required in FR-009
- **Validation**: textstat library scoring during content generation

### Principle II: Accuracy ✅
- **Requirement**: All technical statements correct, proven, conservative
- **Plan Alignment**: FR-006 requires all claims traceable to official documentation; research phase validates sources
- **Validation**: Manual review against ROS 2, Gazebo, NVIDIA Isaac, OpenAI Whisper official docs

### Principle III: Minimalism ✅
- **Requirement**: Remove anything not directly helping learning
- **Plan Alignment**: 1200-1600 word limit per chapter (FR-002); no filler content allowed
- **Validation**: Word count enforcement; editorial review for unnecessary content

### Principle IV: Speed ✅
- **Requirement**: Full site + embeddings build < 2 minutes on GitHub Actions free tier
- **Plan Alignment**: FR-018 enforces < 2 minute build; no heavy assets or unnecessary dependencies
- **Validation**: GitHub Actions workflow timing; local build performance testing

### Principle V: Free-Tier Only ✅
- **Requirement**: Every component runs at $0/month
- **Plan Alignment**: FR-005 mandates CPU-only code examples; free-tier alternatives documented in FR-010
- **Validation**: No paid API keys, no GPU requirements, all services free-tier

### Principle VI: Source-of-Truth RAG ✅
- **Requirement**: Chatbot answers ONLY from final rendered book text
- **Plan Alignment**: FR-011 ensures clean headings for RAG ingestion; SC-007 validates chatbot citation accuracy
- **Validation**: Content structure review; RAG system integration testing (separate feature)

**Gate Status**: ✅ PASS - All constitutional principles satisfied by plan

## Project Structure

### Documentation (this feature)

```text
specs/001-modules-1-4-chapters/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - technology decisions & source validation
├── data-model.md        # Phase 1 output - content structure & chapter schema
├── quickstart.md        # Phase 1 output - setup instructions for contributors
├── contracts/           # Phase 1 output - content templates & validation rules
│   ├── chapter-template.md
│   ├── code-example-schema.json
│   └── mermaid-diagram-guidelines.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/
├── module1/
│   └── robotic-nervous-system-ros2.md  # Chapter 2: ROS 2 Fundamentals
├── module2/
│   └── digital-twin-gazebo-unity.md    # Chapter 3: Digital Twin Simulation
├── module3/
│   └── ai-robot-brain-nvidia-isaac.md  # Chapter 4: NVIDIA Isaac
└── module4/
    └── vision-language-action-vla.md   # Chapter 5: VLA Systems

src/
├── components/
│   ├── CodeBlock.tsx              # Collapsible code block component
│   └── MermaidDiagram.tsx         # Mermaid diagram wrapper
└── utils/
    └── readability-checker.py      # Flesch-Kincaid validation script

tests/
├── integration/
│   ├── test_build.py              # Docusaurus build validation
│   ├── test_links.py              # Broken link detection
│   └── test_code_examples.py      # Python code snippet execution
└── unit/
    └── test_readability.py        # Readability scoring tests

.github/
└── workflows/
    └── build-and-deploy.yml       # CI/CD pipeline (< 2 min build requirement)

sidebars.ts                        # Docusaurus sidebar configuration
docusaurus.config.js               # Docusaurus configuration (onBrokenLinks: 'throw')
package.json                       # Node.js dependencies
```

**Structure Decision**: Static documentation site structure with module-based organization. Each module gets its own directory under `docs/` for clear separation. Custom React components in `src/components/` handle MDX rendering (code blocks, Mermaid diagrams). Testing infrastructure validates build performance, readability, and code correctness. GitHub Actions workflow automates deployment to GitHub Pages.

## Complexity Tracking

*No constitutional violations - this section remains empty.*

## Phase 0: Research & Technology Decisions

### Research Tasks

#### Task 1: ROS 2 Documentation & Code Patterns
**Objective**: Validate ROS 2 content accuracy and identify free-tier code examples
**Sources**:
- Official ROS 2 documentation (https://docs.ros.org/en/humble/)
- rclpy API reference
- ROS 2 Design Patterns documentation
**Deliverables**:
- List of 5-8 runnable rclpy code examples (publisher-subscriber, services, actions)
- Mermaid diagram structures for ROS 2 architecture and data flow
- Free-tier setup instructions (Ubuntu/WSL2, Docker alternatives)

#### Task 2: Gazebo & Unity Integration Research
**Objective**: Confirm Gazebo/Unity simulation approaches and CPU-only examples
**Sources**:
- Gazebo official documentation (https://gazebosim.org/docs)
- Unity Robotics Hub documentation
- ROS-Gazebo bridge documentation
**Deliverables**:
- 5-8 CPU-only Gazebo simulation examples (sensor simulation, basic physics)
- Mermaid diagrams for sensor pipeline and simulation workflows
- Free-tier cloud alternatives (if local simulation insufficient)

#### Task 3: NVIDIA Isaac Platform Research
**Objective**: Identify free-tier Isaac access and document perception-planning-action loop
**Sources**:
- NVIDIA Isaac documentation (https://developer.nvidia.com/isaac)
- Isaac Sim cloud trial details
- Isaac ROS packages documentation
**Deliverables**:
- 5-8 Isaac examples accessible via free-tier cloud trials
- Mermaid diagrams for Isaac architecture and AI-robot integration
- Clear documentation of free-tier limits and alternatives

#### Task 4: Vision-Language-Action (VLA) Systems
**Objective**: Document VLA pipeline with local Whisper examples
**Sources**:
- OpenAI Whisper documentation (https://github.com/openai/whisper)
- VLA research papers (overview only, no advanced research)
- Computer vision + NLP integration patterns
**Deliverables**:
- 5-8 runnable VLA examples (local Whisper for speech, OpenCV for vision)
- Mermaid diagrams for full VLA loop (human command → robot action)
- CPU-only execution instructions (no GPU required)

#### Task 5: Readability & Content Quality Standards
**Objective**: Establish Flesch-Kincaid scoring methodology and validation process
**Sources**:
- textstat Python library documentation
- Flesch-Kincaid readability formula
- Technical writing best practices for grade 10-12 audience
**Deliverables**:
- Python script for automated readability scoring
- Target metrics (Flesch-Kincaid grade 10-12, word count 1200-1600)
- Editorial guidelines for simplicity and clarity

#### Task 6: Docusaurus MDX & Mermaid Integration
**Objective**: Validate Mermaid rendering and collapsible code block implementation
**Sources**:
- Docusaurus MDX documentation
- Mermaid.js syntax reference
- Docusaurus plugin ecosystem
**Deliverables**:
- Code block component with collapsible functionality
- Mermaid diagram rendering configuration
- Build optimization for < 2 minute constraint

### Research Output: `research.md`

*See `specs/001-modules-1-4-chapters/research.md` for consolidated findings.*

## Phase 1: Design & Content Structure

### Data Model: Content Schema

#### Entity: Chapter
**Purpose**: Self-contained learning module for one course module

**Attributes**:
- `title` (string, required): Chapter title matching module name
- `module_number` (integer, 1-4): Sequential module identifier
- `word_count` (integer, 1200-1600): Content length validation
- `mermaid_diagrams` (array, 4-6 items): Visual representations
- `code_examples` (array, 5-8 items): Runnable Python snippets
- `navigation_next` (string, required): Link to next chapter
- `readability_score` (float, 10-12): Flesch-Kincaid grade level

**Validation Rules**:
- Word count MUST be between 1200-1600
- Mermaid diagram count MUST be between 4-6
- Code example count MUST be between 5-8
- Readability score MUST be between 10-12
- All code examples MUST execute without errors on free-tier infrastructure

#### Entity: Mermaid Diagram
**Purpose**: Visual representation of architecture, workflow, or data flow

**Attributes**:
- `type` (enum): flowchart | sequence | class | state
- `title` (string, required): Diagram purpose description
- `mermaid_syntax` (text, required): Valid Mermaid.js code
- `render_validation` (boolean): Confirms successful rendering in Docusaurus

**Validation Rules**:
- Mermaid syntax MUST be valid (no rendering errors)
- Diagram MUST be relevant to chapter learning objectives
- Title MUST clearly describe diagram content

#### Entity: Code Example
**Purpose**: Runnable Python/rclpy snippet demonstrating concept

**Attributes**:
- `language` (string, "python"): Programming language
- `description` (string, required): What the code demonstrates
- `code` (text, required): Executable Python code
- `dependencies` (array): Required packages (e.g., ["rclpy", "numpy"])
- `free_tier_compatible` (boolean, true): Confirms CPU-only execution
- `expected_output` (text): Sample output for validation

**Validation Rules**:
- Code MUST execute without errors on Python 3.8+
- Dependencies MUST be free-tier installable (pip, no paid APIs)
- Code MUST NOT require GPU
- Inline comments MUST explain key steps

### API Contracts (Content Templates)

*See `specs/001-modules-1-4-chapters/contracts/` for detailed templates.*

#### Contract 1: Chapter Template (`contracts/chapter-template.md`)
Defines standard structure for all chapters:
- Frontmatter (title, module number, next link)
- Introduction (2-3 paragraphs, learning objectives)
- Core Concepts (3-5 sections with headings)
- Mermaid Diagrams (4-6, evenly distributed)
- Code Examples (5-8, with explanations)
- Summary & Next Steps
- References (official documentation links)

#### Contract 2: Code Example Schema (`contracts/code-example-schema.json`)
JSON schema for code example validation:
```json
{
  "type": "object",
  "required": ["language", "description", "code", "free_tier_compatible"],
  "properties": {
    "language": {"type": "string", "enum": ["python"]},
    "description": {"type": "string", "minLength": 10},
    "code": {"type": "string", "minLength": 50},
    "dependencies": {"type": "array", "items": {"type": "string"}},
    "free_tier_compatible": {"type": "boolean", "const": true},
    "expected_output": {"type": "string"}
  }
}
```

#### Contract 3: Mermaid Diagram Guidelines (`contracts/mermaid-diagram-guidelines.md`)
Best practices for diagram creation:
- Use flowcharts for architecture overviews
- Use sequence diagrams for communication patterns
- Keep diagrams simple (max 10-12 nodes)
- Include descriptive labels on all connections
- Validate rendering in Docusaurus before commit

### Quickstart Guide

*See `specs/001-modules-1-4-chapters/quickstart.md` for full setup instructions.*

**Summary**:
1. Clone repository and checkout `001-modules-1-4-chapters` branch
2. Install Node.js 18+ and Python 3.8+
3. Run `npm install` to install Docusaurus dependencies
4. Install Python validation tools: `pip install textstat pytest`
5. Create chapter file in `docs/moduleX/` using chapter template
6. Write content following readability and structure guidelines
7. Add Mermaid diagrams (validate syntax locally)
8. Write Python code examples (test execution: `pytest tests/integration/test_code_examples.py`)
9. Check readability: `python src/utils/readability-checker.py docs/moduleX/your-chapter.md`
10. Validate build: `npm run build` (must complete in < 2 minutes)
11. Update `sidebars.ts` with chapter title and path
12. Commit changes with descriptive message

## Phase 2: Implementation Tasks

*This section is populated by `/sp.tasks` command - NOT created by `/sp.plan`.*

**Placeholder**: Task breakdown for implementation will include:
- Task 1: Create Chapter 2 (Module 1: ROS 2)
- Task 2: Create Chapter 3 (Module 2: Digital Twin)
- Task 3: Create Chapter 4 (Module 3: NVIDIA Isaac)
- Task 4: Create Chapter 5 (Module 4: VLA Systems)
- Task 5: Update sidebars.ts with all chapter titles
- Task 6: Implement readability validation script
- Task 7: Create code example execution tests
- Task 8: Configure GitHub Actions workflow
- Task 9: Validate build performance (< 2 min)
- Task 10: Final quality review and documentation

## Testing Strategy

### Validation Checks (Automated)

#### 1. Build Validation
**Test**: `npm run build`
**Success Criteria**:
- Build completes without errors
- Build time < 2 minutes
- Zero broken links (onBrokenLinks: 'throw')
**Implementation**: GitHub Actions workflow with timing measurement

#### 2. Readability Scoring
**Test**: `python src/utils/readability-checker.py docs/**/*.md`
**Success Criteria**:
- All chapters score Flesch-Kincaid grade 10-12
- Word count per chapter: 1200-1600
**Implementation**: Custom Python script using textstat library

#### 3. Code Example Execution
**Test**: `pytest tests/integration/test_code_examples.py`
**Success Criteria**:
- All 20-32 code examples execute without errors
- All examples use free-tier infrastructure (CPU-only)
- Dependencies installable via pip (no paid APIs)
**Implementation**: pytest fixtures extracting and running code from markdown

#### 4. Mermaid Diagram Rendering
**Test**: Manual visual inspection + Docusaurus build
**Success Criteria**:
- All 16-24 diagrams render correctly
- No syntax errors in Mermaid code
**Implementation**: Visual review during content creation; build validation

#### 5. Link Integrity
**Test**: Docusaurus build with `onBrokenLinks: 'throw'`
**Success Criteria**:
- Zero broken internal links
- All navigation links functional
**Implementation**: Docusaurus built-in link checker

### Manual Validation Checks

#### 1. Content Accuracy Review
**Process**: Compare chapter content against official documentation sources
**Reviewers**: Technical SME or peer review
**Criteria**: All technical claims match official ROS 2, Gazebo, NVIDIA Isaac, Whisper docs

#### 2. Constitutional Compliance Audit
**Process**: Verify alignment with 6 core principles
**Checklist**:
- [ ] Simplicity: First-read clarity achieved
- [ ] Accuracy: All claims traceable to official sources
- [ ] Minimalism: No unnecessary content
- [ ] Speed: Build time < 2 minutes validated
- [ ] Free-Tier: All examples CPU-only, no paid services
- [ ] Source-of-Truth RAG: Clean headings for chatbot ingestion

#### 3. RAG Integration Readiness
**Process**: Validate content structure for RAG chatbot
**Criteria**:
- Clean heading hierarchy (H1, H2, H3)
- Each section independently citable
- No ambiguous section titles
**Note**: Actual RAG chatbot integration is separate feature

## Post-Design Constitution Re-Check

### Principle I: Simplicity ✅
**Status**: PASS
**Evidence**: Readability validation script enforces Flesch-Kincaid grade 10-12; chapter template requires active voice and concrete examples

### Principle II: Accuracy ✅
**Status**: PASS
**Evidence**: Research phase validates all sources against official documentation; manual accuracy review process defined

### Principle III: Minimalism ✅
**Status**: PASS
**Evidence**: 1200-1600 word limit enforced; editorial review process removes unnecessary content

### Principle IV: Speed ✅
**Status**: PASS
**Evidence**: GitHub Actions workflow enforces < 2 minute build; no heavy assets or unnecessary dependencies planned

### Principle V: Free-Tier Only ✅
**Status**: PASS
**Evidence**: All code examples CPU-only; no paid APIs, GPU requirements, or services in plan; free-tier alternatives documented

### Principle VI: Source-of-Truth RAG ✅
**Status**: PASS
**Evidence**: Clean heading structure enforced in chapter template; content designed for RAG ingestion; citation capability preserved

**Final Gate Status**: ✅ PASS - All constitutional principles remain satisfied after design phase

## Architectural Decisions Requiring ADR

### Decision 1: Docusaurus for Static Site Generation
**Context**: Need fast, maintainable documentation site with MDX support
**Decision**: Use Docusaurus 3.x as static site generator
**Rationale**:
- Built-in MDX support for React components (code blocks, Mermaid)
- Excellent build performance (meets < 2 min constraint)
- Free GitHub Pages deployment
- Strong community support and documentation
**Alternatives Considered**:
- MkDocs Material: Lacks MDX/React component support
- VitePress: Less mature ecosystem, harder MDX integration
- Custom Next.js: Overkill for static content, slower build times
**Recommendation**: Document in ADR if this decision impacts other features (e.g., RAG chatbot integration)

### Decision 2: CPU-Only Code Examples
**Context**: Free-tier constraint prohibits GPU usage
**Decision**: All Python code examples must run on CPU without GPU dependencies
**Rationale**:
- Aligns with Constitution Principle V (Free-Tier Only)
- Ensures universal accessibility (students without GPUs can learn)
- Reduces complexity and setup barriers
**Alternatives Considered**:
- GPU-optional examples: Adds complexity, violates free-tier principle
- Cloud GPU trials: Ephemeral, violates $0/month constraint
**Recommendation**: Document in ADR due to impact on code quality and learning outcomes

### Decision 3: Flesch-Kincaid Grade 10-12 as Readability Standard
**Context**: Need measurable simplicity metric for Constitution Principle I
**Decision**: Enforce Flesch-Kincaid readability grade 10-12 for all chapters
**Rationale**:
- Aligns with intermediate engineering student target audience
- Objective, automatable metric (textstat library)
- Industry-standard readability measure
**Alternatives Considered**:
- Gunning Fog Index: Less familiar to writers
- SMOG Index: Similar results, less tool support
- Manual review only: Not scalable, subjective
**Recommendation**: Document in ADR if other content features use different readability standards

## Dependencies & Integration Points

### Internal Dependencies
- **Docusaurus Configuration**: Requires `docusaurus.config.js` with `onBrokenLinks: 'throw'`
- **Sidebar Configuration**: `sidebars.ts` must be updated with chapter titles and paths
- **GitHub Actions Workflow**: `.github/workflows/build-and-deploy.yml` must enforce < 2 min build
- **Constitution**: `.specify/memory/constitution.md` governs all design decisions

### External Dependencies
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/ (content accuracy)
- **Gazebo Documentation**: https://gazebosim.org/docs (simulation examples)
- **NVIDIA Isaac Documentation**: https://developer.nvidia.com/isaac (AI-robot integration)
- **OpenAI Whisper Documentation**: https://github.com/openai/whisper (VLA examples)
- **Mermaid.js**: Diagram rendering in Docusaurus MDX
- **textstat Library**: Python readability scoring

### Integration Points (Separate Features)
- **RAG Chatbot**: Will ingest these chapters for question answering (separate feature)
- **Better Auth**: User authentication for personalization (bonus feature, separate implementation)
- **Urdu Translation**: Per-chapter language toggle (bonus feature, separate implementation)

## Risk Analysis

### Risk 1: Build Time Exceeds 2 Minutes
**Likelihood**: Medium
**Impact**: High (constitutional violation)
**Mitigation**:
- Monitor build times continuously during development
- Optimize asset loading (lazy-load Mermaid diagrams)
- Minimize dependencies (remove unused Docusaurus plugins)
- Use GitHub Actions caching for node_modules

### Risk 2: Code Examples Fail on Free-Tier Infrastructure
**Likelihood**: Low
**Impact**: High (breaks student learning experience)
**Mitigation**:
- Test all code examples on CPU-only machines before commit
- Document exact dependency versions in requirements.txt
- Provide troubleshooting guides for common setup issues
- Use Docker containers for consistent execution environment

### Risk 3: Content Fails Readability Validation
**Likelihood**: Medium
**Impact**: Medium (constitutional violation, content rework required)
**Mitigation**:
- Run readability checker during writing (not just at end)
- Provide editorial guidelines and examples for grade 10-12 writing
- Iterative refinement based on textstat scoring
- Peer review before final commit

### Risk 4: Official Documentation Changes During Development
**Likelihood**: Low
**Impact**: Medium (content accuracy compromised)
**Mitigation**:
- Snapshot official documentation URLs with archive.org
- Include "last verified" dates in reference sections
- Monitor official doc repos for major updates
- Plan for content update cycle post-hackathon

### Risk 5: Mermaid Diagrams Fail to Render
**Likelihood**: Low
**Impact**: Medium (visual learning aids broken)
**Mitigation**:
- Validate Mermaid syntax locally before commit (Mermaid Live Editor)
- Include fallback text descriptions for all diagrams
- Test rendering in multiple browsers (Chrome, Firefox, Safari)
- Monitor Docusaurus build logs for Mermaid errors

## Success Metrics

### Quantitative Metrics
- ✅ **Chapter Completion**: 4/4 chapters delivered (Modules 1-4)
- ✅ **Word Count**: 5000-6400 words total (1200-1600 per chapter)
- ✅ **Mermaid Diagrams**: 16-24 diagrams total (4-6 per chapter)
- ✅ **Code Examples**: 20-32 examples total (5-8 per chapter)
- ✅ **Readability**: All chapters score Flesch-Kincaid grade 10-12
- ✅ **Build Time**: < 2 minutes on GitHub Actions free tier
- ✅ **Broken Links**: 0 (Docusaurus build passes with onBrokenLinks: 'throw')
- ✅ **Free-Tier Compliance**: 100% of code examples CPU-only, no paid services

### Qualitative Metrics
- ✅ **Constitutional Compliance**: All 6 principles satisfied (audit checklist complete)
- ✅ **Content Accuracy**: All technical claims verified against official documentation
- ✅ **RAG Readiness**: Content structure suitable for chatbot ingestion (clean headings, citeable sections)
- ✅ **Student Learning**: Chapters enable students to run code examples and build understanding independently

### Hackathon Scoring Alignment
- ✅ **Mandatory Features** (100 base points): Clean Docusaurus UI, zero broken links, RAG-ready content structure
- ✅ **Bonus Features** (+200 points potential): Reusable Claude Code subagents/skills documented in PHRs (this plan demonstrates spec-driven workflow)

## Next Steps

1. **Immediate**: Run `/sp.tasks` to generate task breakdown for implementation
2. **Phase 0**: Execute research tasks to validate technology decisions (research.md)
3. **Phase 1**: Create content structure artifacts (data-model.md, contracts/, quickstart.md)
4. **Phase 2**: Implement chapters following task list
5. **Validation**: Run automated tests (build, readability, code execution, links)
6. **Review**: Constitutional compliance audit and manual content accuracy review
7. **Deployment**: Merge to main branch, trigger GitHub Pages deployment

## Appendix: File Paths

- **Plan**: `specs/001-modules-1-4-chapters/plan.md` (this file)
- **Spec**: `specs/001-modules-1-4-chapters/spec.md`
- **Research**: `specs/001-modules-1-4-chapters/research.md` (Phase 0 output)
- **Data Model**: `specs/001-modules-1-4-chapters/data-model.md` (Phase 1 output)
- **Quickstart**: `specs/001-modules-1-4-chapters/quickstart.md` (Phase 1 output)
- **Contracts**: `specs/001-modules-1-4-chapters/contracts/` (Phase 1 output)
- **Tasks**: `specs/001-modules-1-4-chapters/tasks.md` (Phase 2, `/sp.tasks` command)
- **Constitution**: `.specify/memory/constitution.md`
- **Chapter Files**:
  - `docs/module1/robotic-nervous-system-ros2.md`
  - `docs/module2/digital-twin-gazebo-unity.md`
  - `docs/module3/ai-robot-brain-nvidia-isaac.md`
  - `docs/module4/vision-language-action-vla.md`
