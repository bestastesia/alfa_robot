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
      ["architecture.html", "架构驾驶舱"],
      ["status.html", "状态看板"]
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
    const featured = ["motion_internal_interfaces", "robot_motion_scene_service", "alfa_robot_moveit_config", "alfa_robot_execution_bridge"];
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

  function renderArchitecture() {
    const architecture = data.architecture;
    shell(
      "系统架构驾驶舱",
      "面向项目交付、跨部门协作和长期维护的双臂运控能力地图。",
      `
      <section class="architecture-command">
        <div class="architecture-command-copy">
          <p class="eyebrow">${escapeHtml(architecture.headline.kicker)}</p>
          <h2>${escapeHtml(architecture.headline.title)}</h2>
          <p>${escapeHtml(architecture.headline.summary)}</p>
          <div class="tag-row architecture-badges">${architecture.headline.badges.map((item) => pill(item, "cyan")).join("")}</div>
        </div>
        <div class="architecture-metrics">
          ${architecture.capabilityMetrics.map((item) => `
            <article>
              <strong>${escapeHtml(item.value)}</strong>
              <span>${escapeHtml(item.label)}</span>
              <small>${escapeHtml(item.note)}</small>
            </article>
          `).join("")}
        </div>
      </section>
      <div class="architecture-viewbar" role="toolbar" aria-label="架构视图切换">
        <div>
          <button class="architecture-view active" type="button" data-view="executive">交付视角</button>
          <button class="architecture-view" type="button" data-view="engineering">工程视角</button>
        </div>
        <span id="architecture-view-hint">突出能力闭环、事实源和保障边界</span>
      </div>
      <div class="architecture-view-scope" data-scope="executive">
      <section class="section architecture-overview-section">
        <div class="section-head architecture-section-head">
          <div>
            <p class="eyebrow">Operational Loop</p>
            <h2>一条闭环解释系统如何工作</h2>
          </div>
          <p>任务只向前流动，状态与证据闭环返回；任何实现都不能绕过契约直接控制下一层。</p>
        </div>
        <div class="control-loop">
          ${architecture.controlLoop.map((item, index) => `
            <article class="control-node" data-node="${escapeHtml(item.id)}">
              <span class="control-node-index">${String(index + 1).padStart(2, "0")}</span>
              <div class="control-node-icon">${escapeHtml(item.label.slice(0, 1))}</div>
              <h3>${escapeHtml(item.label)}</h3>
              <strong>${escapeHtml(item.owner)}</strong>
              <p>${escapeHtml(item.detail)}</p>
              <code>${escapeHtml(item.contract)}</code>
              ${index < architecture.controlLoop.length - 1 ? '<i class="control-arrow">→</i>' : '<i class="control-arrow feedback-arrow">↺</i>'}
            </article>
          `).join("")}
        </div>
      </section>
      <section class="architecture-dual-grid">
        <article class="section truth-panel">
          <div class="section-head">
            <p class="eyebrow">Single Source of Truth</p>
            <h2>四类事实，只有一个权威出口</h2>
          </div>
          <div class="truth-chain">
            ${architecture.truthChain.map((item, index) => `
              <div class="truth-item">
                <span>${String(index + 1).padStart(2, "0")}</span>
                <div><h3>${escapeHtml(item.title)}</h3><strong>${escapeHtml(item.owner)}</strong><p>${escapeHtml(item.content)}</p></div>
              </div>
            `).join("")}
          </div>
        </article>
        <article class="section assurance-panel">
          <div class="section-head">
            <p class="eyebrow">Engineering Assurance</p>
            <h2>甲方关心的不只是“能动”</h2>
          </div>
          <div class="guardrail-grid">
            ${architecture.guardrails.map((item) => `
              <div class="guardrail-item"><span>${escapeHtml(item.icon)}</span><div><h3>${escapeHtml(item.title)}</h3><p>${escapeHtml(item.text)}</p></div></div>
            `).join("")}
          </div>
        </article>
      </section>
      </div>
      <div class="architecture-view-scope" data-scope="engineering">
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">System Boundaries</p>
          <h2>六层能力栈与不可跨越的边界</h2>
          <p>每层只拥有一种核心责任；向下调用能力，向上报告结果。</p>
        </div>
        <div class="architecture-stack">
          ${architecture.layers.map((layer, index) => `
            <article class="architecture-layer">
              <span class="layer-index">${escapeHtml(layer.index)}</span>
              <div class="layer-title">
                <h3>${escapeHtml(layer.name)}</h3>
                <code>${escapeHtml(layer.modules)}</code>
              </div>
              <div>
                <strong>拥有</strong>
                <p>${escapeHtml(layer.owns)}</p>
              </div>
              <div>
                <strong>禁止</strong>
                <p>${escapeHtml(layer.mustNot)}</p>
              </div>
              ${index < architecture.layers.length - 1 ? '<span class="layer-arrow">↓</span>' : ''}
            </article>
          `).join("")}
        </div>
        <div class="tool-lane">
          <strong>侧向工具域</strong>
          <span>robot_motion_tools / scripts/ik_benchmark / Rerun / system_tests</span>
          <em>只调用公开服务，不得成为任何生产包的依赖</em>
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Operating Rules</p>
          <h2>四条不可破坏的工程规则</h2>
        </div>
        <div class="principle-grid">
          ${architecture.principles.map((item) => `
            <article class="principle-item">
              <h3>${escapeHtml(item.title)}</h3>
              <p>${escapeHtml(item.rule)}</p>
              <small>${escapeHtml(item.prevents)}</small>
            </article>
          `).join("")}
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Ownership Matrix</p>
          <h2>一个概念只能有一个负责人</h2>
        </div>
        <div class="table-scroll">
          <table class="ownership-table">
            <thead><tr><th>概念</th><th>唯一负责人</th><th>公开 Interface</th><th>明确禁止</th></tr></thead>
            <tbody>
              ${architecture.ownership.map((row) => `
                <tr>
                  <th>${escapeHtml(row.concern)}</th>
                  <td>${escapeHtml(row.owner)}</td>
                  <td><code>${escapeHtml(row.interface)}</code></td>
                  <td>${escapeHtml(row.forbidden)}</td>
                </tr>
              `).join("")}
            </tbody>
          </table>
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Promotion Path</p>
          <h2>调试代码如何合法进入正式系统</h2>
          <p>不是禁止快速脚本，而是禁止它跳过中间门槛直接成为生产依赖。</p>
        </div>
        <div class="promotion-flow">
          ${architecture.promotion.map((item, index) => `
            <article class="promotion-step">
              <span>${index + 1}</span>
              <h3>${escapeHtml(item.stage)}</h3>
              <code>${escapeHtml(item.location)}</code>
              <p>${escapeHtml(item.gate)}</p>
            </article>
          `).join('<b>→</b>')}
        </div>
      </section>
      <section class="section">
        <div class="section-head">
          <p class="eyebrow">Migration</p>
          <h2>从当前仓库到目标架构</h2>
        </div>
        <div class="migration-list">
          ${architecture.migration.map((item) => `
            <article>
              <strong>${escapeHtml(item.phase)}</strong>
              <p>${escapeHtml(item.result)}</p>
            </article>
          `).join("")}
        </div>
      </section>
      </div>`
    );

    const viewButtons = $all(".architecture-view");
    const scopes = $all(".architecture-view-scope");
    const hint = $("#architecture-view-hint");
    viewButtons.forEach((button) => {
      button.addEventListener("click", () => {
        const view = button.dataset.view;
        viewButtons.forEach((item) => item.classList.toggle("active", item === button));
        scopes.forEach((scope) => scope.classList.toggle("view-muted", scope.dataset.scope !== view));
        hint.textContent = view === "executive"
          ? "突出能力闭环、事实源和保障边界"
          : "突出依赖方向、唯一负责人和代码晋升门槛";
        scopes.find((scope) => scope.dataset.scope === view)?.scrollIntoView({ behavior: "smooth", block: "start" });
      });
    });
  }

  function boot() {
    const page = document.body.dataset.page;
    if (page === "flows") renderFlows();
    else if (page === "packages") renderPackages();
    else if (page === "package") renderPackageDetail();
    else if (page === "architecture") renderArchitecture();
    else if (page === "status") renderStatus();
    else renderIndex();
  }

  window.addEventListener("DOMContentLoaded", boot);
})();
