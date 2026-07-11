(function () {
  const data = window.SYSTEM_PORTAL_DATA;

  function $(selector, root = document) {
    return root.querySelector(selector);
  }

  function $all(selector, root = document) {
    return Array.from(root.querySelectorAll(selector));
  }

  function escapeHtml(value) {
    return String(value ?? "")
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#039;");
  }

  function list(items) {
    if (!items || !items.length) return "<span class=\"muted\">暂无</span>";
    return `<ul class="compact-list">${items.map((item) => `<li>${escapeHtml(item)}</li>`).join("")}</ul>`;
  }

  function pill(text, tone = "") {
    return `<span class="pill ${tone}">${escapeHtml(text)}</span>`;
  }

  function statusTone(maturity) {
    const map = {
      stable: "green",
      active: "blue",
      transitional: "orange",
      lab: "purple",
      utility: "cyan",
      "active-risky": "red",
      "legacy-support": "gray",
      external: "gray",
      "mock-or-early": "red"
    };
    return map[maturity] || "blue";
  }

  function packageById(id) {
    return data.packages.find((pkg) => pkg.id === id || pkg.name === id);
  }

  function nav() {
    const current = location.pathname.split("/").pop() || "index.html";
    const links = [
      ["index.html", "系统总览"],
      ["flows.html", "流程地图"],
      ["packages.html", "包职责"],
      ["status.html", "状态看板"],
      ["architecture.html", "架构评审"],
      ["target_architecture.html", "目标架构"],
      ["refactor_plan.html", "重构路线"]
    ];
    return `
      <nav class="top-nav">
        <a class="brand" href="index.html">
          <span class="brand-mark">AR</span>
          <span>
            <strong>${escapeHtml(data.meta.title)}</strong>
            <small>${escapeHtml(data.meta.updated)}</small>
          </span>
        </a>
        <div class="nav-links">
          ${links.map(([href, label]) => `<a class="${current === href ? "active" : ""}" href="${href}">${label}</a>`).join("")}
        </div>
      </nav>`;
  }

  function shell(title, subtitle, content) {
    document.title = `${title} · ${data.meta.title}`;
    const app = $("#app");
    app.innerHTML = `
      ${nav()}
      <main class="page">
        <section class="hero">
          <div>
            <p class="eyebrow">System Portal</p>
            <h1>${escapeHtml(title)}</h1>
            <p>${escapeHtml(subtitle)}</p>
          </div>
          <div class="hero-card">
            <span>更新规则</span>
            <strong>${escapeHtml(data.meta.updateRule)}</strong>
            <em>${escapeHtml(data.meta.branchHint)}</em>
          </div>
        </section>
        ${content}
      </main>
      <footer class="footer">
        <span>ALFA Robot System Portal</span>
        <span>数据源：<code>docs/system_portal/assets/data.js</code></span>
      </footer>`;
  }

  function renderExternalActors() {
    return `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Layer 1</p>
          <h2>系统和外界怎么交互</h2>
          <p>先用外部角色理解系统边界，再进入内部包和流程。</p>
        </div>
        <div class="actor-flow">
          ${data.externalActors.map((actor, index) => `
            <article class="actor-card">
              <span class="step-index">${String(index + 1).padStart(2, "0")}</span>
              <h3>${escapeHtml(actor.name)}</h3>
              <p>${escapeHtml(actor.role)}</p>
              <div class="tag-row">${actor.interfaces.map((item) => pill(item)).join("")}</div>
            </article>
          `).join("")}
        </div>
      </section>`;
  }

  function renderRecommended() {
    return `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Quick Start</p>
          <h2>推荐阅读路径</h2>
        </div>
        <div class="card-grid three">
          ${data.recommendedEntrypoints.map((entry) => `
            <a class="big-link" href="${escapeHtml(entry.href)}">
              <span>${escapeHtml(entry.title)}</span>
              <p>${escapeHtml(entry.hint)}</p>
            </a>
          `).join("")}
        </div>
      </section>`;
  }

  function renderPackagePreview() {
    const featured = ["robot_motion_interfaces", "robot_motion_scene_service", "alfa_robot_moveit_config", "alfa_robot_execution_bridge"];
    return `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Core Packages</p>
          <h2>核心包入口</h2>
        </div>
        <div class="card-grid">
          ${featured.map((id) => renderPackageCard(packageById(id))).join("")}
        </div>
      </section>`;
  }

  function renderFlowSummary() {
    return `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Layer 2</p>
          <h2>内部主流程</h2>
          <p>每条流程都能进入详情页，看数据形式、包责任和阶段状态。</p>
        </div>
        <div class="flow-grid">
          ${data.systemFlows.map((flow) => `
            <a class="flow-card" href="flows.html#${escapeHtml(flow.id)}">
              <h3>${escapeHtml(flow.title)}</h3>
              <p>${escapeHtml(flow.summary)}</p>
              <div class="mini-flow">${flow.stages.slice(0, 5).map((stage) => `<span>${escapeHtml(stage.name)}</span>`).join("<b>→</b>")}</div>
            </a>
          `).join("")}
        </div>
      </section>`;
  }

  function renderIndex() {
    shell(
      "系统总览",
      "一眼看清外界、运控系统、执行层、机器人本体和可视化之间的关系。",
      renderExternalActors() + renderFlowSummary() + renderPackagePreview() + renderRecommended()
    );
  }

  function renderFlowDetail(flow) {
    return `
      <section class="section flow-detail" id="${escapeHtml(flow.id)}">
        <div class="section-head">
          <p class="eyebrow">Workflow</p>
          <h2>${escapeHtml(flow.title)}</h2>
          <p>${escapeHtml(flow.summary)}</p>
        </div>
        <div class="timeline">
          ${flow.stages.map((stage, index) => `
            <article class="timeline-item">
              <span class="timeline-index">${index + 1}</span>
              <div>
                <h3>${escapeHtml(stage.name)}</h3>
                <dl class="data-dl">
                  <dt>负责/相关包</dt><dd>${escapeHtml(stage.owner)}</dd>
                  <dt>输入/消费</dt><dd>${escapeHtml(stage.data)}</dd>
                  <dt>输出/贡献</dt><dd>${escapeHtml(stage.output)}</dd>
                </dl>
              </div>
            </article>
          `).join("")}
        </div>
      </section>`;
  }

  function renderFlows() {
    shell(
      "流程地图",
      "从业务流程进入，查看每一步由哪些包负责、输入什么、产出什么。",
      `<section class="section index-strip">
        ${data.systemFlows.map((flow) => `<a href="#${escapeHtml(flow.id)}">${escapeHtml(flow.title)}</a>`).join("")}
      </section>` +
      data.systemFlows.map(renderFlowDetail).join("")
    );
  }

  function renderPackageCard(pkg) {
    if (!pkg) return "";
    return `
      <a class="package-card" href="package.html?id=${encodeURIComponent(pkg.id)}" data-layer="${escapeHtml(pkg.layer)}" data-maturity="${escapeHtml(pkg.maturity)}" data-search="${escapeHtml(`${pkg.name} ${pkg.layer} ${pkg.status} ${pkg.responsibility}`.toLowerCase())}">
        <div class="card-top">
          ${pill(pkg.layer)}
          ${pill(pkg.status, statusTone(pkg.maturity))}
        </div>
        <h3>${escapeHtml(pkg.name)}</h3>
        <p>${escapeHtml(pkg.responsibility)}</p>
        <span class="more">查看输入输出和状态 →</span>
      </a>`;
  }

  function renderPackages() {
    const layers = [...new Set(data.packages.map((pkg) => pkg.layer))];
    shell(
      "包职责",
      "搜索或按层级查看每个 ROS2 包/核心模块的责任、输入、输出和当前风险。",
      `
      <section class="section controls">
        <input id="package-search" class="search" placeholder="搜索包名、职责、状态，例如 scene / IK / mock / MoveIt" />
        <div class="filter-row">
          <button class="filter active" data-filter="all">全部</button>
          ${layers.map((layer) => `<button class="filter" data-filter="${escapeHtml(layer)}">${escapeHtml(layer)}</button>`).join("")}
        </div>
      </section>
      <section class="section">
        <div class="card-grid packages" id="package-grid">
          ${data.packages.map(renderPackageCard).join("")}
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Non ROS Assets</p>
          <h2>非 ROS 包但会影响流程的资产</h2>
        </div>
        <div class="card-grid three">
          ${data.nonRosAssets.map((asset) => `
            <article class="info-card">
              ${pill(asset.status)}
              <h3>${escapeHtml(asset.name)}</h3>
              <p>${escapeHtml(asset.role)}</p>
              <small>${escapeHtml(asset.notes)}</small>
            </article>
          `).join("")}
        </div>
      </section>`
    );
    bindPackageFilters();
  }

  function bindPackageFilters() {
    const search = $("#package-search");
    const buttons = $all(".filter");
    const cards = $all(".package-card");
    let activeLayer = "all";

    function apply() {
      const q = (search.value || "").trim().toLowerCase();
      cards.forEach((card) => {
        const layerOk = activeLayer === "all" || card.dataset.layer === activeLayer;
        const searchOk = !q || card.dataset.search.includes(q);
        card.hidden = !(layerOk && searchOk);
      });
    }

    search.addEventListener("input", apply);
    buttons.forEach((button) => {
      button.addEventListener("click", () => {
        buttons.forEach((item) => item.classList.remove("active"));
        button.classList.add("active");
        activeLayer = button.dataset.filter;
        apply();
      });
    });
  }

  function renderPackageDetail() {
    const params = new URLSearchParams(location.search);
    const pkg = packageById(params.get("id")) || data.packages[0];
    const relatedFlows = data.systemFlows.filter((flow) =>
      flow.stages.some((stage) => stage.owner.toLowerCase().includes(pkg.name.toLowerCase()) || stage.owner.toLowerCase().includes(pkg.id.toLowerCase().replaceAll("_", " ")))
    );
    shell(
      pkg.name,
      pkg.responsibility,
      `
      <section class="section detail-layout">
        <aside class="detail-side">
          ${pill(pkg.layer)}
          ${pill(pkg.status, statusTone(pkg.maturity))}
          <h2>${escapeHtml(pkg.name)}</h2>
          <p>${escapeHtml(pkg.responsibility)}</p>
          <a class="back-link" href="packages.html">← 返回包列表</a>
        </aside>
        <div class="detail-main">
          <div class="detail-card">
            <h3>输入 / 消费</h3>
            ${list(pkg.consumes)}
          </div>
          <div class="detail-card">
            <h3>输出 / 贡献</h3>
            ${list(pkg.produces)}
          </div>
          <div class="detail-card">
            <h3>关键位置</h3>
            ${list(pkg.keyFiles)}
          </div>
          <div class="detail-card">
            <h3>状态 / 风险 / 未完成</h3>
            ${list(pkg.statusNotes)}
          </div>
          <div class="detail-card">
            <h3>相关流程</h3>
            ${relatedFlows.length ? relatedFlows.map((flow) => `<a class="inline-link" href="flows.html#${escapeHtml(flow.id)}">${escapeHtml(flow.title)}</a>`).join("") : "<span class=\"muted\">暂无显式关联；可从流程页按职责查找。</span>"}
          </div>
        </div>
      </section>`
    );
  }

  function renderStatus() {
    const byMaturity = data.packages.reduce((acc, pkg) => {
      acc[pkg.maturity] = acc[pkg.maturity] || [];
      acc[pkg.maturity].push(pkg);
      return acc;
    }, {});
    shell(
      "状态看板",
      "快速找出哪些包是稳定接口、哪些仍是实验、过渡或 mock 状态。",
      `
      <section class="section">
        <div class="legend-grid">
          ${data.statusLegend.map((item) => `
            <article class="legend-card">
              ${pill(item.label, item.color)}
              <h3>${escapeHtml(item.key)}</h3>
              <p>${escapeHtml(item.meaning)}</p>
            </article>
          `).join("")}
        </div>
      </section>
      <section class="section">
        <div class="status-columns">
          ${Object.entries(byMaturity).map(([maturity, packages]) => `
            <article class="status-column">
              <h3>${escapeHtml(maturity)} <span>${packages.length}</span></h3>
              ${packages.map((pkg) => `<a href="package.html?id=${encodeURIComponent(pkg.id)}">${escapeHtml(pkg.name)}<small>${escapeHtml(pkg.status)}</small></a>`).join("")}
            </article>
          `).join("")}
        </div>
      </section>`
    );
  }

  // ---------- 架构评审页面（数据源：assets/architecture_data.js） ----------

  const archData = window.ARCHITECTURE_REVIEW_DATA;

  const ARCH_TONES = {
    ok: { stroke: "rgba(147, 197, 253, 0.55)", fill: "rgba(147, 197, 253, 0.10)", text: "#eef4ff" },
    good: { stroke: "rgba(110, 231, 183, 0.65)", fill: "rgba(110, 231, 183, 0.10)", text: "#d7fbe9" },
    warn: { stroke: "rgba(253, 186, 116, 0.70)", fill: "rgba(253, 186, 116, 0.10)", text: "#ffedd5" },
    bad: { stroke: "rgba(252, 165, 165, 0.75)", fill: "rgba(252, 165, 165, 0.10)", text: "#fee2e2" },
    dim: { stroke: "rgba(203, 213, 225, 0.35)", fill: "rgba(203, 213, 225, 0.06)", text: "#9fb0cb" }
  };

  const EDGE_KINDS = {
    ok: { stroke: "rgba(147, 197, 253, 0.6)", dash: "", width: 1.6 },
    good: { stroke: "rgba(110, 231, 183, 0.8)", dash: "", width: 2 },
    warn: { stroke: "rgba(253, 186, 116, 0.85)", dash: "7 4", width: 2 },
    bad: { stroke: "rgba(252, 165, 165, 0.95)", dash: "7 4", width: 2.4 },
    dim: { stroke: "rgba(203, 213, 225, 0.3)", dash: "3 4", width: 1.2 }
  };

  function severityTone(severity) {
    return { critical: "red", high: "orange", medium: "blue", low: "gray" }[severity] || "gray";
  }

  function verifiedPill(verified) {
    if (verified === "CONFIRMED") return pill("已复核确认", "green");
    if (verified === "PARTIAL") return pill("部分成立", "orange");
    return pill("待复核", "gray");
  }

  function rectAnchor(node, tx, ty) {
    // 从 node 中心指向 (tx, ty) 的射线与矩形边框交点，用于贴边画箭头。
    const cx = node.x + node.w / 2;
    const cy = node.y + node.h / 2;
    const dx = tx - cx;
    const dy = ty - cy;
    if (dx === 0 && dy === 0) return { x: cx, y: cy };
    const sx = dx !== 0 ? (node.w / 2) / Math.abs(dx) : Infinity;
    const sy = dy !== 0 ? (node.h / 2) / Math.abs(dy) : Infinity;
    const s = Math.min(sx, sy);
    return { x: cx + dx * s, y: cy + dy * s };
  }

  function renderGraphSvg(graph) {
    const nodes = graph.nodes.map((n) => ({ h: 46, ...n }));
    const byId = Object.fromEntries(nodes.map((n) => [n.id, n]));
    const edges = (graph.edges || []).map((edge) => {
      const from = byId[edge.from];
      const to = byId[edge.to];
      if (!from || !to) return "";
      const kind = EDGE_KINDS[edge.kind] || EDGE_KINDS.ok;
      const fromCenter = { x: from.x + from.w / 2, y: from.y + from.h / 2 };
      const toCenter = { x: to.x + to.w / 2, y: to.y + to.h / 2 };
      const a = rectAnchor(from, toCenter.x, toCenter.y);
      const b = rectAnchor(to, fromCenter.x, fromCenter.y);
      const midX = (a.x + b.x) / 2;
      const midY = (a.y + b.y) / 2;
      const label = edge.label
        ? `<text class="arch-edge-label" x="${midX}" y="${midY - 5}" text-anchor="middle">${escapeHtml(edge.label)}</text>`
        : "";
      return `
        <line x1="${a.x}" y1="${a.y}" x2="${b.x}" y2="${b.y}"
          stroke="${kind.stroke}" stroke-width="${kind.width}"
          ${kind.dash ? `stroke-dasharray="${kind.dash}"` : ""}
          marker-end="url(#arch-arrow-${edge.kind || "ok"})" />
        ${label}`;
    }).join("");

    const boxes = nodes.map((n) => {
      const tone = ARCH_TONES[n.tone] || ARCH_TONES.ok;
      const sub = n.sub
        ? `<text x="${n.x + n.w / 2}" y="${n.y + n.h / 2 + 14}" text-anchor="middle" class="arch-node-sub">${escapeHtml(n.sub)}</text>`
        : "";
      const mainY = n.sub ? n.y + n.h / 2 - 1 : n.y + n.h / 2 + 4;
      return `
        <rect x="${n.x}" y="${n.y}" width="${n.w}" height="${n.h}" rx="10"
          fill="${tone.fill}" stroke="${tone.stroke}" stroke-width="1.4" />
        <text x="${n.x + n.w / 2}" y="${mainY}" text-anchor="middle" class="arch-node-label" fill="${tone.text}">${escapeHtml(n.label)}</text>
        ${sub}`;
    }).join("");

    const markers = Object.entries(EDGE_KINDS).map(([key, kind]) => `
      <marker id="arch-arrow-${key}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="${kind.stroke}" />
      </marker>`).join("");

    return `
      <div class="arch-graph-wrap">
        <svg class="arch-graph" viewBox="0 0 ${graph.width} ${graph.height}" role="img" aria-label="${escapeHtml(graph.caption || "依赖图")}">
          <defs>${markers}</defs>
          ${edges}
          ${boxes}
        </svg>
        ${graph.caption ? `<p class="arch-caption">${escapeHtml(graph.caption)}</p>` : ""}
        ${graph.legend ? `<div class="tag-row arch-legend">${graph.legend.map((item) => pill(item.label, item.tone)).join("")}</div>` : ""}
      </div>`;
  }

  function renderFindingCard(finding) {
    const category = (archData.categories || []).find((c) => c.key === finding.category) || { label: finding.category, tone: "gray" };
    return `
      <article class="finding-card" data-category="${escapeHtml(finding.category)}" data-severity="${escapeHtml(finding.severity)}"
        data-search="${escapeHtml(`${finding.id} ${finding.title} ${(finding.packages || []).join(" ")} ${finding.problem}`.toLowerCase())}">
        <div class="card-top">
          <span class="finding-id">${escapeHtml(finding.id)}</span>
          ${pill(category.label, category.tone)}
          ${pill(finding.severity, severityTone(finding.severity))}
          ${verifiedPill(finding.verified)}
        </div>
        <h3>${escapeHtml(finding.title)}</h3>
        <dl class="data-dl">
          <dt>位置</dt><dd>${(finding.packages || []).map((p) => `<code>${escapeHtml(p)}</code>`).join(" ")}</dd>
          <dt>证据</dt><dd>${escapeHtml(finding.evidence)}</dd>
          <dt>为什么是问题</dt><dd>${escapeHtml(finding.problem)}</dd>
          <dt>实际后果</dt><dd>${escapeHtml(finding.consequence)}</dd>
          <dt>建议动作</dt><dd>${escapeHtml(finding.recommendation)}</dd>
          ${finding.risk ? `<dt>迁移风险</dt><dd>${escapeHtml(finding.risk)}</dd>` : ""}
        </dl>
      </article>`;
  }

  function bindFindingFilters() {
    const search = $("#finding-search");
    const buttons = $all(".filter[data-cat-filter]");
    const sevButtons = $all(".filter[data-sev-filter]");
    const cards = $all(".finding-card");
    let activeCategory = "all";
    let activeSeverity = "all";

    function apply() {
      const q = (search.value || "").trim().toLowerCase();
      cards.forEach((card) => {
        const catOk = activeCategory === "all" || card.dataset.category === activeCategory;
        const sevOk = activeSeverity === "all" || card.dataset.severity === activeSeverity;
        const searchOk = !q || card.dataset.search.includes(q);
        card.hidden = !(catOk && sevOk && searchOk);
      });
    }

    search.addEventListener("input", apply);
    buttons.forEach((button) => {
      button.addEventListener("click", () => {
        buttons.forEach((item) => item.classList.remove("active"));
        button.classList.add("active");
        activeCategory = button.dataset.catFilter;
        apply();
      });
    });
    sevButtons.forEach((button) => {
      button.addEventListener("click", () => {
        sevButtons.forEach((item) => item.classList.remove("active"));
        button.classList.add("active");
        activeSeverity = button.dataset.sevFilter;
        apply();
      });
    });
  }

  function renderArchitectureReview() {
    const usedCategories = [...new Set(archData.findings.map((f) => f.category))];
    const categoryButtons = usedCategories.map((key) => {
      const category = archData.categories.find((c) => c.key === key) || { label: key };
      return `<button class="filter" data-cat-filter="${escapeHtml(key)}">${escapeHtml(category.label)}</button>`;
    }).join("");

    shell(
      "架构评审：现状问题全景",
      archData.meta.reviewSubtitle,
      `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Verdict</p>
          <h2>${escapeHtml(archData.verdict.headline)}</h2>
        </div>
        <div class="card-grid three">
          ${archData.verdict.points.map((point) => `
            <article class="info-card">
              <h3>${escapeHtml(point.title)}</h3>
              <p>${escapeHtml(point.body)}</p>
            </article>`).join("")}
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Dependency Graph</p>
          <h2>当前真实依赖与启动关系</h2>
          <p>基于 package.xml、CMakeLists 与 launch 文件核实，不是理想图。</p>
        </div>
        ${renderGraphSvg(archData.currentGraph)}
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">God Package</p>
          <h2>${escapeHtml(archData.godPackage.name)} 解剖</h2>
          <p>${escapeHtml(archData.godPackage.statLine)}</p>
        </div>
        <div class="card-grid">
          ${archData.godPackage.groups.map((group) => `
            <article class="info-card">
              ${pill(group.verdict, group.tone)}
              <h3>${escapeHtml(group.title)}</h3>
              <p>${escapeHtml(group.note)}</p>
              <ul class="compact-list">${group.items.map((item) => `<li><code>${escapeHtml(item)}</code></li>`).join("")}</ul>
            </article>`).join("")}
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Findings</p>
          <h2>问题清单（${archData.findings.length} 项，均含文件级证据）</h2>
          <p>每项都经过独立复核代理确认或标注为部分成立；可按类别、严重度过滤。</p>
        </div>
        <div class="controls">
          <input id="finding-search" class="search" placeholder="搜索问题、包名、文件，例如 moveit / ik_benchmark / launch" />
          <div class="filter-row">
            <button class="filter active" data-cat-filter="all">全部类别</button>
            ${categoryButtons}
          </div>
          <div class="filter-row">
            <button class="filter active" data-sev-filter="all">全部严重度</button>
            ${["critical", "high", "medium", "low"].map((s) => `<button class="filter" data-sev-filter="${s}">${s}</button>`).join("")}
          </div>
        </div>
        <div class="finding-grid">
          ${archData.findings.map(renderFindingCard).join("")}
        </div>
      </section>`
    );
    bindFindingFilters();
  }

  function renderTargetArchitecture() {
    shell(
      "目标架构与依赖方向",
      archData.meta.targetSubtitle,
      `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Target</p>
          <h2>目标包架构与允许的依赖方向</h2>
          <p>箭头方向 = 允许的依赖方向；任何逆向箭头都应视为架构回归。</p>
        </div>
        ${renderGraphSvg(archData.targetGraph)}
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Rules</p>
          <h2>依赖铁律</h2>
        </div>
        <div class="card-grid">
          ${archData.dependencyRules.map((rule, index) => `
            <article class="info-card">
              <span class="step-index">${String(index + 1).padStart(2, "0")}</span>
              <h3>${escapeHtml(rule.rule)}</h3>
              <p>${escapeHtml(rule.detail)}</p>
            </article>`).join("")}
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Responsibilities</p>
          <h2>包职责变化矩阵</h2>
        </div>
        <div class="arch-table-wrap">
          <table class="arch-table">
            <thead><tr><th>包</th><th>现状职责</th><th>目标职责</th><th>变化</th></tr></thead>
            <tbody>
              ${archData.responsibilityMatrix.map((row) => `
                <tr>
                  <td><code>${escapeHtml(row.pkg)}</code></td>
                  <td>${escapeHtml(row.today)}</td>
                  <td>${escapeHtml(row.target)}</td>
                  <td>${pill(row.change, row.tone)}</td>
                </tr>`).join("")}
            </tbody>
          </table>
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Relocations</p>
          <h2>需要搬家 / 隔离 / 删除的代码</h2>
        </div>
        <div class="arch-table-wrap">
          <table class="arch-table">
            <thead><tr><th>代码 / 配置</th><th>现在位置</th><th>去向</th><th>原因</th><th>批次</th></tr></thead>
            <tbody>
              ${archData.relocations.map((row) => `
                <tr>
                  <td>${escapeHtml(row.what)}</td>
                  <td><code>${escapeHtml(row.from)}</code></td>
                  <td>${pill(row.to, row.tone)}</td>
                  <td>${escapeHtml(row.reason)}</td>
                  <td>${escapeHtml(row.order)}</td>
                </tr>`).join("")}
            </tbody>
          </table>
        </div>
      </section>`
    );
  }

  function renderRefactorPlan() {
    shell(
      "重构路线：增量迁移计划",
      archData.meta.planSubtitle,
      `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Guardrails</p>
          <h2>不可破坏的行为边界</h2>
          <p>每一批迁移都必须保持这些外部契约不变，否则回滚。</p>
        </div>
        <div class="card-grid three">
          ${archData.guardrails.map((item) => `
            <article class="info-card">
              <h3>${escapeHtml(item.title)}</h3>
              <p>${escapeHtml(item.body)}</p>
            </article>`).join("")}
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Plan</p>
          <h2>迁移批次（每批独立可回滚）</h2>
        </div>
        <div class="timeline">
          ${archData.phases.map((phase, index) => `
            <article class="timeline-item">
              <span class="timeline-index">${index + 1}</span>
              <div>
                <h3>${escapeHtml(phase.title)} ${pill(phase.riskLabel, phase.riskTone)}</h3>
                <p class="muted">${escapeHtml(phase.goal)}</p>
                <dl class="data-dl">
                  <dt>动作</dt><dd><ul class="compact-list">${phase.steps.map((step) => `<li>${escapeHtml(step)}</li>`).join("")}</ul></dd>
                  <dt>验证口径</dt><dd><ul class="compact-list">${phase.verification.map((v) => `<li>${escapeHtml(v)}</li>`).join("")}</ul></dd>
                  <dt>退出条件</dt><dd>${escapeHtml(phase.exit)}</dd>
                </dl>
              </div>
            </article>`).join("")}
        </div>
      </section>`
    );
  }

  function renderArchPending(title) {
    shell(
      title,
      "评审数据尚未生成。",
      `
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Pending</p>
          <h2>评审数据生成中</h2>
          <p>本页面需要 <code>assets/architecture_data.js</code>；架构评审完成后该文件会随评审结论一起提交。</p>
        </div>
      </section>`
    );
  }

  function boot() {
    const page = document.body.dataset.page;
    if (page === "flows") renderFlows();
    else if (page === "packages") renderPackages();
    else if (page === "package") renderPackageDetail();
    else if (page === "status") renderStatus();
    else if (page === "architecture") archData ? renderArchitectureReview() : renderArchPending("架构评审：现状问题全景");
    else if (page === "target-architecture") archData ? renderTargetArchitecture() : renderArchPending("目标架构与依赖方向");
    else if (page === "refactor-plan") archData ? renderRefactorPlan() : renderArchPending("重构路线：增量迁移计划");
    else renderIndex();
  }

  window.addEventListener("DOMContentLoaded", boot);
})();
