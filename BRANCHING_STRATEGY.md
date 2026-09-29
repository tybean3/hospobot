# Hospobot Repository Architecture: BaseNav vs SemanticNav

This document defines the Git branching model for managing **BaseNav** (stable foundation) and **SemanticNav** (experimental high-level AI/semantic features).

---

## 1. Branch Hierarchy

```
       (Foundation: Hardware, AMCL, Nav2, WebDash, Maps)
       BaseNav (or main) ────●─────────●─────────●─────────► [Always Clean & Stable]
                              \         \
                        merge  \         \ merge
                                ▼         ▼
       SemanticNav ───────────●────●────●────●─────────────► [AI / Vision / Semantic]
                                   ▲         ▲
                                   │         │
                           (Semantic Features: YOLO, VLM,
                            LLM Planners, Semantic Maps)
```

| Branch | Purpose | What lives here | Merge Policy |
| :--- | :--- | :--- | :--- |
| **`BaseNav`** | Stable Foundation | Motor control (ODrive CAN), 2D LiDAR correlative auto-localization, Nav2 base navigation, keep-out masks, web dashboards, base launch files. | **Upstream:** Merges changes *into* `SemanticNav`. Never receives merges from `SemanticNav`. |
| **`SemanticNav`** | Experimental Intelligence | Visual detection (YOLO, DepthAI / OAK-D spatial features), Vision-Language Models (VLM), LLM task reasoning, room/semantic semantic maps, semantic costmap layers. | **Downstream:** Periodically pulls all updates from `BaseNav`. Never merged back into `BaseNav`. |

---

## 2. Daily Workflow

### Scenario A: Making a Fix or Improvement to Base Navigation
*(e.g., motor tuning, keepout zone adjustment, web dash feature, AMCL parameter tuning)*

1. Switch to `BaseNav`:
   ```bash
   git checkout BaseNav
   ```
2. Make your edits and test on the robot.
3. Commit and push to GitHub:
   ```bash
   git add .
   git commit -m "fix(base): update costmap inflation radius"
   git push origin BaseNav
   ```
4. **Propagate to SemanticNav** so it gets the fix immediately:
   ```bash
   git checkout SemanticNav
   git merge BaseNav -m "chore: sync BaseNav updates into SemanticNav"
   git push origin SemanticNav
   ```
   *(Alternatively, just run `./scripts/sync_base_to_semantic.sh --push`)*

---

### Scenario B: Developing Semantic Navigation Features
*(e.g., semantic object tags, camera perception nodes, LLM planner)*

1. Switch to `SemanticNav`:
   ```bash
   git checkout SemanticNav
   ```
2. Develop semantic packages (recommended: keep semantic nodes in dedicated ROS 2 packages such as `src/hospobot_semantic/`).
3. Commit and push:
   ```bash
   git add .
   git commit -m "feat(semantic): integrate visual object goal publisher"
   git push origin SemanticNav
   ```
4. **DO NOT** merge `SemanticNav` into `BaseNav`.

---

## 3. Automated Sync Tool

A convenience script is provided in `scripts/sync_base_to_semantic.sh`:

```bash
# Sync local BaseNav changes into SemanticNav:
./scripts/sync_base_to_semantic.sh

# Sync and immediately push to GitHub:
./scripts/sync_base_to_semantic.sh --push
```

---

## 4. Best Practices to Prevent Merge Conflicts

1. **Modular ROS 2 Packages**:
   - Place all semantic nodes, models, and scripts into their own ROS 2 package (e.g., `src/hospobot/src/hospobot_semantic/` or separate workspace package).
   - This ensures `BaseNav` files are rarely modified in `SemanticNav`, making merges completely clean and conflict-free.
2. **Launch File Modularity**:
   - Keep `hospobot_launch.py` in `BaseNav` focused on base navigation.
   - In `SemanticNav`, create a `semantic_launch.py` that includes `hospobot_launch.py` via `IncludeLaunchDescription` and adds the semantic nodes.
