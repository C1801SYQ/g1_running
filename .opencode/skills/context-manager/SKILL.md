---
name: context-manager
description: "Hybrid memory system for AI agents. Stores learned knowledge persistently and manages conversation context efficiently to reduce token usage. Use when: storing important information between sessions, retrieving previously learned concepts, maintaining user preferences, or optimizing token consumption in long conversations."
license: GNU General Public License v3.0
compatibility: OpenCode
metadata:
  audience: general
  use_case: knowledge management, context optimization
---

# Context Manager Skill

A clean, file-based hybrid memory system that combines **long-term structured knowledge** with **daily conversation logs** to dramatically reduce token usage.

## Solution

**4-layer memory architecture:**

| Layer      | Scope                | Storage              | Purpose                      |
| ---------- | -------------------- | -------------------- | ---------------------------- |
| Short-term | Last 3-5 turns       | Context window       | Maintain flow                |
| Mid-term   | Daily logs           | `logs/YYYY-MM-DD.md` | Full daily history           |
| Long-term  | Structured knowledge | `knowledge/` folders | Curated, searchable insights |
| Buffer     | Working drafts       | `scratchpad/`        | Temporary context/WIP notes  |

## When to Use

- User mentions important information to remember
- Starting a new session — retrieve relevant past context
- **Drafting complex ideas that aren't yet ready for the knowledge base (Scratchpad)**
- Learning new concepts, APIs, or workflows
- Noting user preferences or project-specific patterns
- Summarizing to reduce token usage in long conversations

## Storage Location

**Project-relative**: `./context-memory/`

```markdown
./context-memory/
├── INDEX.md # Master knowledge index
├── scratchpad/ # WIP and temporary data
├── knowledge/ # Long-term structured knowledge
│ ├── concepts/
│ ├── technology/
│ ├── workflows/
│ ├── preferences/
│ └── tools/
└── logs/ # Daily conversation logs
```

## Workflow

### 1. Learning New Knowledge

When user shares important information:

```bash
# Create knowledge file in appropriate category
cat > ./context-memory/knowledge/technology/api-name.md << 'EOF'
# API Name

## Metadata
- **Learned**: $(date '+%Y-%m-%d %H:%M')
- **Source**: Conversation with user
- **Importance**: ⭐⭐⭐⭐
- **Tags**: #api #integration

## Summary
Brief description of what was learned.

## Key Details
- Point 1
- Point 2

## Related
- [[linked-knowledge-file]]
EOF
```

### 2. Retrieving Knowledge

```bash
# Keyword search
grep -r "keyword" ./context-memory/knowledge/

# Semantic search (if colgrep is installed)
colgrep "search query" ./context-memory/

# Read a file
cat ./context-memory/knowledge/technology/api-name.md
```

### 3. Daily Logging

```bash
echo "## $(date '+%H:%M') Topic" >> ./context-memory/logs/$(date '+%Y-%m-%d').md
echo "- User: [summary]" >> ./context-memory/logs/$(date '+%Y-%m-%d').md
echo "- AI: [response summary]" >> ./context-memory/logs/$(date '+%Y-%m-%d').md
```

### 4. Context Optimization (Token Savings)

Instead of loading full history, retrieve relevant snippets:

```bash
# Get recent context
tail -50 ./context-memory/logs/$(date '+%Y-%m-%d').md
```

## File Templates

### Knowledge Entry

```markdown
# [Title]

## Metadata

- **Learned**: YYYY-MM-DD HH:MM
- **Source**: [conversation/external]
- **Importance**: ⭐⭐ (1-5 stars)
- **Tags**: #tag1 #tag2
- **Updated**: YYYY-MM-DD HH:MM

## Summary

[50-100 word overview]

## Details

[Detailed content]

## Key Takeaways

- Point 1
- Point 2

## Related

- [[related-file]]
```

### Daily Log Entry

```markdown
## HH:MM Topic

- User: [message summary]
- AI: [response summary]
- Key points: [important insights]
```

## Usage Examples

### Storing User Preference

```bash
cat > ./context-memory/knowledge/preferences/user-coffee.md << 'EOF'
# Coffee Preference

## Metadata
- **Learned**: YYYY-MM-DD
- **Importance**: ⭐⭐⭐⭐

## Summary
User prefers medium roast with oat milk, no sugar.

## Details
- Temperature: 65°C
- Favorite shop: [shop name]
EOF
```

### Retrieving Past Context

```bash
# Find all mentions of a topic
grep -r "api" ./context-memory/knowledge/
colgrep "authentication patterns" ./context-memory/
```

### Starting New Session

```bash
# Check for relevant knowledge
ls ./context-memory/knowledge/
cat ./context-memory/INDEX.md
tail -20 ./context-memory/logs/YYYY-MM-DD.md
```

## Token Optimization

| Approach             | Tokens     | Use Case            |
| -------------------- | ---------- | ------------------- |
| Full history         | 25,000+    | When needed         |
| Last 5 turns         | ~2,500     | Normal conversation |
| Relevant retrieval   | ~500       | Specific questions  |
| **With this system** | **~2,700** | **89% savings**     |

## Maintenance

### Weekly

- Review and consolidate duplicate entries
- Update INDEX.md with new entries

### Monthly

- Archive or summarize old logs
- Clean up outdated knowledge

### As Needed

- Update existing entries with new info
- Add related links between concepts

## Integration

Works with other skills:

- **colgrep**: Semantic search across memory
- **spreadsheet**: Store structured data
- **humanizer**: Polish knowledge entries

## Troubleshooting

**Can't find knowledge?**

- Check INDEX.md for overview
- Use `grep` or `colgrep` with keywords
- Verify correct category folder

**Storage growing too large?**

- Run weekly consolidation
- Summarize old logs
- Archive inactive entries

**Context not loading?**

- Check `logs/YYYY-MM-DD.md` exists
- Verify recent entries have summaries
