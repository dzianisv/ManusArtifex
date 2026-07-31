# Agent instructions

## Notion is the project's memory

**Use the Notion MCP server. Do not hand-roll HTTP calls.**

Already configured at `~/.config/opencode/opencode.json` as `@notionhq/notion-mcp-server`.
If your runtime has no MCP, add it:

```json
{"mcp": {"notion": {"type": "local",
  "command": ["npx", "-y", "@notionhq/notion-mcp-server"],
  "environment": {"NOTION_TOKEN": "<token>"}}}}
```

| Page | What it is | ID |
|---|---|---|
| [Robo Hand](https://app.notion.com/p/Robo-Hand-3abac25eb49f80ea93c6e29618f29456) | Index | `3abac25eb49f80ea93c6e29618f29456` |
| [GOALS](https://app.notion.com/p/GOALS-3aeac25eb49f81b89480d586136a4809) | What done looks like | `3aeac25e-b49f-81b8-9480-d586136a4809` |
| [STATUS](https://app.notion.com/p/STATUS-3aeac25eb49f81b6923dea8de6b9fb73) | What is true today | `3aeac25e-b49f-81b6-923d-ea8de6b9fb73` |
| [BACKLOG](https://app.notion.com/p/3abac25eb49f805f907cd38ca11f7a79) | Future work (database) | page `3abac25e-b49f-805f-907c-d38ca11f7a79`, db `3aeac25e-b49f-81ab-99f6-e9ef242f9cf8` |
| [WORKLOG](https://app.notion.com/p/WORKLOG-3aeac25eb49f816e9947c7416e63a1e7) | What happened, by date | `3aeac25e-b49f-816e-9947-c7416e63a1e7` |
| [DESIGN](https://app.notion.com/p/3acac25eb49f80d09e1efc8fc4576df8) | Decisions and why | `3acac25e-b49f-80d0-9e1e-fc8fc4576df8` |
| [AGENT](https://app.notion.com/p/AGENT-how-to-use-these-pages-3aeac25eb49f81aca684e5310fea59ed) | This file, in Notion | `3aeac25e-b49f-81ac-a684-e5310fea59ed` |
| [REFERENCE](https://app.notion.com/p/REFERENCE-3aeac25eb49f810e9995fb54a2750e2c) | Hardware facts, repair steps | `3aeac25e-b49f-810e-9995-fb54a2750e2c` |
| └ [hardware discovery](https://app.notion.com/p/3abac25eb49f81b7a85ef95d7ff549bd) | Verified commands and outputs | `3abac25e-b49f-81b7-a85e-f95d7ff549bd` |
| └ [servo 5 repair](https://app.notion.com/p/3acac25eb49f8149bc30e5457c77b1a0) | Parts, prices, runbook | `3acac25e-b49f-8149-bc30-e5457c77b1a0` |
| └ [diagnose shoulder servo](https://app.notion.com/p/3aeac25eb49f80fbacffd64c94f24b27) | Pen-line test | `3aeac25e-b49f-80fb-acff-d64c94f24b27` |

## Start of session

1. Read STATUS. It is the only page that says what is true now.
2. Read GOALS.
3. Query BACKLOG for `State = Next`. Pick work with `Owner = Agent`.
4. Re-check any hardware fault STATUS lists as open. Do not trust it. Servo 5 flipped from dead to alive to dead across three sessions.

## End of session

Three writes. All three, every time.

1. **STATUS** — delete the old blocks, write new ones. Overwrite. Never append. Update the date line.
2. **WORKLOG** — append one `## YYYY-MM-DD` heading and bullets. Never edit an old entry.
3. **BACKLOG** — update `State` on rows you moved. Delete finished rows; their result goes in WORKLOG.

## Rules

- Only STATUS says "currently". Every other page links to it instead.
- BACKLOG holds future work only. No history.
- WORKLOG is append-only.
- DESIGN has no status in it.
- One fact, one page. If you write the same fact twice, one copy will go stale.

## Writing style

Short sentences. Plain English. Facts and numbers. No filler words.

Bad: "We conducted a comprehensive investigation which revealed that the observer was potentially unreliable."
Good: "The camera could not see the wrist. 337 changed px against a 1,243 noise floor."

## MCP tools

The ones you need, by name:

| Job | Tool |
|---|---|
| Read a page as markdown | `API-retrieve-page-markdown` |
| Rewrite a page's content | `API-update-page-markdown` |
| List child pages | `API-get-block-children` |
| Append blocks | `API-patch-block-children` |
| Edit one block | `API-update-a-block` |
| Delete a block | `API-delete-a-block` |
| Query the backlog | `API-query-data-source` |
| Add a backlog row | `API-post-page` with `parent: {database_id}` |
| New page | `API-post-page` |
| Rename a page | `API-patch-page` |
| Move a page | `API-move-page` with `parent: {type, page_id}` |

Prefer the markdown tools for STATUS and WORKLOG. Block JSON is only needed for
databases and single-block edits.

## Hardware

Read [docs/safety.md](docs/safety.md) before commanding any motion.

- The servos report nothing. No position, no current. The camera is the only sensor.
- A servo was destroyed here by one bad command. Every command is checked before it is sent.
- Never leave torque in a joint that is not moving. Stop after 3 attempts.
- Run `robohand markers` before trusting any vision result.
- Two signals or it did not happen. Camera and battery sag. Either one alone has lied.
