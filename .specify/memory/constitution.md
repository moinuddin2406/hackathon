# Physical AI & Humanoid Robotics — Essentials Constitution

<!--
Sync Impact Report (2025-12-07)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Version Change: INITIAL → 1.0.0
Rationale: Initial constitution creation for hackathon project

Modified Principles: N/A (initial creation)
Added Sections:
  - Core Principles (6 principles defined)
  - Project Scope & Architecture
  - Technical Constraints & Deployment
  - Mandatory Features
  - Bonus Features
  - Success Criteria
  - Governance

Removed Sections: N/A

Templates Requiring Updates:
  ✅ .specify/templates/spec-template.md (verified alignment)
  ✅ .specify/templates/plan-template.md (verified alignment)
  ✅ .specify/templates/tasks-template.md (verified alignment)
  ✅ .specify/templates/phr-template.prompt.md (verified alignment)
  ✅ .specify/templates/adr-template.md (verified alignment)

Follow-up TODOs: None
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
-->

## Project Overview

**Project Name**: Physical AI & Humanoid Robotics — Essentials
**Project Type**: AI-native interactive textbook + free-tier RAG chatbot
**Deployment**: GitHub Pages (must be live and publicly accessible)
**Hackathon Deadline**: 30 Nov 2025, 6 PM (Pakistan Standard Time)

**Core Purpose**:
Create the world's first fully AI/spec-driven, minimalist, high-quality, open-access textbook that teaches the Panaversity Physical AI & Humanoid Robotics course in the fastest and clearest way possible — complete with an embedded RAG chatbot that answers ONLY from the book's content.

## Core Principles

### I. Simplicity

Every sentence MUST be understandable on first read by an intermediate engineering student.

**Rationale**: Learning velocity is maximized when cognitive load is minimized. Complex jargon, nested concepts, and unclear explanations create friction that slows comprehension. By enforcing first-read clarity, we ensure the textbook serves its primary function: teaching effectively and quickly.

**Non-negotiable rules**:
- Use active voice and concrete examples
- Define technical terms immediately upon introduction
- Break complex concepts into digestible chunks
- No assumed knowledge beyond intermediate engineering fundamentals

### II. Accuracy

All technical statements MUST be correct, proven, and conservative (no hype, no speculation).

**Rationale**: Misinformation compounds over time and damages learner trust. In a rapidly evolving field like Physical AI, it's critical to distinguish between established facts, current best practices, and speculative futures. Conservative accuracy builds credibility and lasting value.

**Non-negotiable rules**:
- Every claim must be verifiable through documentation or research
- State limitations and uncertainties explicitly
- No marketing language or unsubstantiated claims
- Cite sources for non-trivial technical statements

### III. Minimalism

Remove anything that does not directly help the reader learn faster.

**Rationale**: Every additional word, image, or section competes for the learner's attention. Minimalism isn't about being sparse—it's about being essential. Every element must justify its existence by accelerating understanding.

**Non-negotiable rules**:
- No decorative content or filler text
- Examples must illustrate core concepts, not edge cases
- Remove redundant explanations
- Prioritize diagrams and code over long prose when clearer

### IV. Speed

Full site + embeddings MUST build in under 2 minutes on GitHub Actions free tier.

**Rationale**: Fast iteration cycles enable rapid improvement. Build speed directly impacts development velocity, testing frequency, and deployment confidence. The 2-minute constraint forces efficient architecture and prevents bloat.

**Non-negotiable rules**:
- Embedding model ≤ 384 dimensions, < 300 MB
- Total repo size < 350 MB (including embeddings)
- Optimize asset loading and chunking strategies
- Monitor build times in CI/CD pipeline

### V. Free-Tier Only

Every single component (hosting, DB, vector store, embeddings, backend) MUST run at $0/month.

**Rationale**: Economic sustainability ensures indefinite availability and removes barriers to forking, experimentation, and global access. Free-tier constraints drive creative efficiency and prevent vendor lock-in.

**Non-negotiable rules**:
- Backend: FastAPI on Fly.io / Render / Cloudflare Workers free tier
- Vector DB: Qdrant Cloud free tier or equivalent
- Frontend: Docusaurus classic template (minimal customization)
- No paid APIs, no credit card requirements
- Document free-tier limits and monitor usage

### VI. Source-of-Truth RAG

Chatbot may ONLY retrieve and answer from the final rendered book text. No external knowledge, no training bleed.

**Rationale**: Answer quality and trustworthiness depend on restricting RAG retrieval to authoritative, reviewed content. Allowing external knowledge introduces unverified information and dilutes the textbook's role as the definitive source.

**Non-negotiable rules**:
- Embeddings generated exclusively from final book markdown
- RAG system must cite exact chapter/section for every answer
- No fallback to general LLM knowledge
- Implement content versioning to sync embeddings with updates

## Project Scope & Architecture

### In Scope: 6 Chapters (Exactly — No More, No Less)

1. **Introduction to Physical AI**
2. **Basics of Humanoid Robotics**
3. **ROS 2 Fundamentals**
4. **Digital Twin Simulation (Gazebo + Isaac Sim)**
5. **Vision-Language-Action Systems**
6. **Capstone: Simple AI-Robot Pipeline**

### Out of Scope

- Advanced research topics beyond course essentials
- Multi-language support (except bonus Urdu feature)
- User-generated content or community forums
- Real-time collaboration features
- Mobile native apps (responsive web only)

### Architecture Stack

- **Frontend**: Docusaurus classic template with custom RAG integration
- **Backend**: FastAPI (Python) on free-tier PaaS
- **Vector Store**: Qdrant Cloud free tier
- **Embeddings**: BGE-small-en-v1.5 or all-MiniLM-L6-v2 (< 384 dim, < 300 MB)
- **Hosting**: GitHub Pages
- **CI/CD**: GitHub Actions (< 2 min build time)

## Technical Constraints & Deployment

### Performance Budgets

- **Build Time**: < 2 minutes on GitHub Actions free tier (hard limit)
- **Bundle Size**: Total repo < 350 MB including embeddings
- **Embedding Model**: ≤ 384 dimensions, < 300 MB total
- **Page Load**: First Contentful Paint < 2s on 3G connection

### Deployment Requirements

- **GitHub Pages**: Must deploy automatically on push to main branch
- **Zero Broken Links**: Docusaurus build with `onBrokenLinks: 'throw'` must pass
- **HTTPS**: Enforced via GitHub Pages
- **Responsive Design**: Mobile, tablet, desktop support

### Security & Privacy

- No user data collection beyond Better Auth (optional bonus feature)
- No tracking scripts or third-party analytics
- Environment variables for all secrets (`.env` with `.env.example`)
- CSP headers configured for XSS protection

## Mandatory Features (100 Base Points)

1. **Clean, responsive Docusaurus UI** with perfect sidebar navigation and dark mode support
2. **"Select text → Ask AI" functionality** that works on desktop and mobile
3. **RAG chatbot in navbar** that cites exact chapter/section for answers
4. **GitHub Actions workflow** that builds and deploys to GitHub Pages in < 2 minutes
5. **Zero broken links** (Docusaurus build must pass with `onBrokenLinks: 'throw'`)

## Bonus Features (Up to +200 Points)

1. **Better Auth signup/signin** with background quiz → content personalization
2. **Per-chapter "Personalise for me" button** (adjusts difficulty based on user profile)
3. **Per-chapter "Urdu میں دکھائیں" toggle** (bilingual support)
4. **Reusable Claude Code subagents/skills** used during creation (document in PHRs)

## Success Criteria (Go / No-Go)

- [ ] Live GitHub Pages URL works with zero errors
- [ ] RAG chatbot answers 20 random course questions correctly and cites sources
- [ ] "Select text → Ask AI" works perfectly on desktop and mobile
- [ ] Build time < 2 minutes on GitHub Actions free tier
- [ ] Total running cost = $0.00/month
- [ ] Demo video < 90 seconds ready
- [ ] All mandatory features implemented and tested

## Governance

### Authority & Precedence

This constitution is the **single source of truth** for all project decisions, architecture, and development practices. In cases of conflict, the constitution supersedes all other documentation, verbal agreements, or prior decisions.

### Compliance Requirements

- Every spec, plan, task, and code change MUST reference which principle/constraint it satisfies
- All pull requests MUST include a compliance check section verifying adherence to principles
- Any deviation from constitutional principles requires a GitHub issue with explicit justification and impact analysis
- Constitution violations discovered during review block merge until resolved

### Amendment Process

The constitution can only be amended via pull request titled `constitution: <brief description>` following this procedure:

1. **Proposal**: Open GitHub issue documenting the proposed change, rationale, and impact
2. **Discussion**: Minimum 24-hour community review period
3. **Draft**: Submit PR updating `.specify/memory/constitution.md` with version bump
4. **Validation**: Run consistency propagation checklist (templates, commands, docs)
5. **Approval**: Requires explicit approval from project maintainers
6. **Migration**: Update all dependent artifacts and create ADR for significant changes

### Version Semantics

- **MAJOR** (X.0.0): Backward-incompatible governance changes, principle removals, or redefinitions
- **MINOR** (x.Y.0): New principles added or materially expanded guidance
- **PATCH** (x.y.Z): Clarifications, wording improvements, typo fixes

### Enforcement & Review

- Weekly constitution compliance audits during development phase
- Post-deployment constitutional review before hackathon submission
- Document all constitutional references in PHRs and ADRs
- Use `.specify/memory/constitution.md` as reference for all planning and implementation

### Commitment

We pledge to deliver the simplest, most accurate, and fastest-learning Physical AI textbook humanity has ever seen — built entirely with free tools, for everyone, forever.

---

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
