const express = require("express");
const http = require("http");
const https = require("https");
const fs = require("fs");
const path = require("path");
const mqtt = require("mqtt");
const sqlite3 = require("sqlite3").verbose();
const { Pool } = require("pg");
const { Server } = require("socket.io");

const HTTPS_PORT = Number(process.env.PORT || process.env.HTTPS_PORT || process.env.HTTP_PORT || 8090);
const REDIRECT_HTTP_PORT = Number(process.env.REDIRECT_HTTP_PORT || 8080);
const TLS_PFX_PATH = process.env.TLS_PFX_PATH || path.join(__dirname, "cert", "ruche-dashboard.pfx");
const TLS_PFX_PASSPHRASE = process.env.TLS_PFX_PASSPHRASE || "ruche-dashboard";
const MQTT_URL = process.env.MQTT_URL || "mqtt://broker.hivemq.com:1883";
const MQTT_TOPIC = process.env.MQTT_TOPIC || "ruches/telemetry";
const MQTT_TOPIC_COMMAND = process.env.MQTT_TOPIC_COMMAND || "ruches/command";
const HISTORY_LIMIT = Number(process.env.HISTORY_LIMIT || 2000);
const DB_PATH = process.env.SQLITE_PATH || path.join(__dirname, "data", "history.sqlite3");
const DATABASE_URL = process.env.DATABASE_URL || "";
const DASH_USER = process.env.DASH_USER || "";
const DASH_PASS = process.env.DASH_PASS || "";
const authEnabled = DASH_USER.length > 0 && DASH_PASS.length > 0;
const usePostgres = DATABASE_URL.length > 0;

const app = express();
app.use(express.json());

let server;
let httpsEnabled = false;
if (fs.existsSync(TLS_PFX_PATH)) {
  const tlsOptions = {
    pfx: fs.readFileSync(TLS_PFX_PATH),
    passphrase: TLS_PFX_PASSPHRASE,
  };
  server = https.createServer(tlsOptions, app);
  httpsEnabled = true;
} else {
  server = http.createServer(app);
}

const io = new Server(server, {
  cors: { origin: "*" },
});

let pgPool = null;
let sqliteDb = null;

app.get("/healthz", (_req, res) => {
  res.json({ ok: true });
});

if (authEnabled) {
  app.use((req, res, next) => {
    if (req.path === "/healthz") return next();
    if (isAuthorized(req.headers.authorization)) return next();
    res.set("WWW-Authenticate", 'Basic realm="Ruches Dashboard"');
    return res.status(401).send("Authentication required");
  });
}

app.use(express.static("public"));
app.get("/api/status", (_req, res) => {
  res.json({
    mqtt_url: MQTT_URL,
    topic: MQTT_TOPIC,
    topic_command: MQTT_TOPIC_COMMAND,
    db_backend: usePostgres ? "postgres" : "sqlite",
    last: state.last,
    history_len: state.history.length,
  });
});

app.post("/api/command", (req, res) => {
  const action = String(req.body?.action || "").toLowerCase();
  let payload = "";

  if (action === "tare") {
    payload = "TARE";
  } else if (action === "cal") {
    const mass = Number(req.body?.mass_g);
    if (!Number.isFinite(mass) || mass <= 0) {
      return res.status(400).json({ ok: false, error: "mass_g invalide" });
    }
    payload = `CAL:${mass.toFixed(2)}`;
  } else {
    return res.status(400).json({ ok: false, error: "action invalide" });
  }

  if (!mqttClient.connected) {
    return res.status(503).json({ ok: false, error: "mqtt deconnecte" });
  }

  mqttClient.publish(MQTT_TOPIC_COMMAND, payload, { qos: 0, retain: false }, (err) => {
    if (err) {
      return res.status(500).json({ ok: false, error: err.message });
    }
    return res.json({ ok: true, topic: MQTT_TOPIC_COMMAND, payload });
  });
});

app.get("/api/history", (_req, res) => {
  res.json({
    history: state.history,
    last: state.last,
  });
});

const state = {
  last: null,
  history: [],
};

function isAuthorized(authHeader) {
  if (!authEnabled) return true;
  if (!authHeader || !authHeader.startsWith("Basic ")) return false;
  try {
    const decoded = Buffer.from(authHeader.slice(6), "base64").toString("utf8");
    return decoded === `${DASH_USER}:${DASH_PASS}`;
  } catch (_e) {
    return false;
  }
}

function nowIso() {
  return new Date().toISOString();
}

function sanitizeTelemetry(payload) {
  if (!payload || typeof payload !== "object") return null;
  if (payload.heartbeat) return null;

  const weight = Number(payload.weight_g);
  const temp = Number(payload.temp_c);
  const hum = Number(payload.hum_pct);
  const batt = Number(payload.batt_pct);
  const rssi = Number(payload.rssi_dbm ?? payload.rssi);
  const packet = Number(payload.packet);
  const alertSignalLost = Number(payload.alert_signal_lost);
  const alertBattLow = Number(payload.alert_batt_low);

  if (!Number.isFinite(weight)) return null;

  return {
    ts: nowIso(),
    packet: Number.isFinite(packet) ? packet : null,
    weight_g: weight,
    temp_c: Number.isFinite(temp) ? temp : null,
    hum_pct: Number.isFinite(hum) ? hum : null,
    batt_pct: Number.isFinite(batt) ? batt : null,
    rssi: Number.isFinite(rssi) ? rssi : null,
    alert_signal_lost: Number.isFinite(alertSignalLost) ? (alertSignalLost ? 1 : 0) : 0,
    alert_batt_low: Number.isFinite(alertBattLow) ? (alertBattLow ? 1 : 0) : 0,
    raw: payload.raw ?? "",
  };
}

function normalizeRows(rows) {
  return rows.map((r) => ({
    ts: r.ts,
    packet: r.packet,
    weight_g: r.weight_g,
    temp_c: r.temp_c,
    hum_pct: r.hum_pct,
    batt_pct: r.batt_pct,
    rssi: r.rssi,
    alert_signal_lost: r.alert_signal_lost || 0,
    alert_batt_low: r.alert_batt_low || 0,
    raw: r.raw || "",
  }));
}

async function initDb() {
  if (usePostgres) {
    pgPool = new Pool({
      connectionString: DATABASE_URL,
      ssl: process.env.PGSSLMODE === "disable" ? false : { rejectUnauthorized: false },
    });
    await pgPool.query(`
      CREATE TABLE IF NOT EXISTS telemetry (
        id BIGSERIAL PRIMARY KEY,
        ts TIMESTAMPTZ NOT NULL,
        packet INTEGER,
        weight_g DOUBLE PRECISION NOT NULL,
        temp_c DOUBLE PRECISION,
        hum_pct DOUBLE PRECISION,
        batt_pct DOUBLE PRECISION,
        rssi INTEGER,
        alert_signal_lost INTEGER DEFAULT 0,
        alert_batt_low INTEGER DEFAULT 0,
        raw TEXT
      )
    `);
    await pgPool.query(`ALTER TABLE telemetry ADD COLUMN IF NOT EXISTS alert_signal_lost INTEGER DEFAULT 0`);
    await pgPool.query(`ALTER TABLE telemetry ADD COLUMN IF NOT EXISTS alert_batt_low INTEGER DEFAULT 0`);
    return;
  }

  fs.mkdirSync(path.dirname(DB_PATH), { recursive: true });
  sqliteDb = new sqlite3.Database(DB_PATH);
  await new Promise((resolve, reject) => {
    sqliteDb.run(
      `CREATE TABLE IF NOT EXISTS telemetry (
         id INTEGER PRIMARY KEY AUTOINCREMENT,
         ts TEXT NOT NULL,
         packet INTEGER,
         weight_g REAL NOT NULL,
         temp_c REAL,
         hum_pct REAL,
         batt_pct REAL,
         rssi INTEGER,
         alert_signal_lost INTEGER DEFAULT 0,
         alert_batt_low INTEGER DEFAULT 0,
         raw TEXT
       )`,
      (err) => (err ? reject(err) : resolve())
    );
  });
  await new Promise((resolve) => {
    sqliteDb.run(`ALTER TABLE telemetry ADD COLUMN alert_signal_lost INTEGER DEFAULT 0`, () => resolve());
  });
  await new Promise((resolve) => {
    sqliteDb.run(`ALTER TABLE telemetry ADD COLUMN alert_batt_low INTEGER DEFAULT 0`, () => resolve());
  });
}

async function loadHistory() {
  if (usePostgres) {
    const rs = await pgPool.query(
      `SELECT ts, packet, weight_g, temp_c, hum_pct, batt_pct, rssi, alert_signal_lost, alert_batt_low, raw
       FROM telemetry
       ORDER BY id DESC
       LIMIT $1`,
      [HISTORY_LIMIT]
    );
    return normalizeRows(rs.rows.reverse());
  }

  const rows = await new Promise((resolve, reject) => {
    sqliteDb.all(
      `SELECT ts, packet, weight_g, temp_c, hum_pct, batt_pct, rssi, alert_signal_lost, alert_batt_low, raw
       FROM telemetry
       ORDER BY id DESC
       LIMIT ?`,
      [HISTORY_LIMIT],
      (err, data) => (err ? reject(err) : resolve(data))
    );
  });
  return normalizeRows(rows.reverse());
}

async function saveTelemetry(row) {
  if (usePostgres) {
    await pgPool.query(
      `INSERT INTO telemetry (ts, packet, weight_g, temp_c, hum_pct, batt_pct, rssi, alert_signal_lost, alert_batt_low, raw)
       VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)`,
      [row.ts, row.packet, row.weight_g, row.temp_c, row.hum_pct, row.batt_pct, row.rssi, row.alert_signal_lost, row.alert_batt_low, row.raw]
    );
    await pgPool.query(
      `DELETE FROM telemetry
       WHERE id NOT IN (SELECT id FROM telemetry ORDER BY id DESC LIMIT $1)`,
      [HISTORY_LIMIT]
    );
    return;
  }

  await new Promise((resolve, reject) => {
    sqliteDb.run(
      `INSERT INTO telemetry (ts, packet, weight_g, temp_c, hum_pct, batt_pct, rssi, alert_signal_lost, alert_batt_low, raw)
       VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`,
      [row.ts, row.packet, row.weight_g, row.temp_c, row.hum_pct, row.batt_pct, row.rssi, row.alert_signal_lost, row.alert_batt_low, row.raw],
      (err) => (err ? reject(err) : resolve())
    );
  });
  await new Promise((resolve, reject) => {
    sqliteDb.run(
      `DELETE FROM telemetry
       WHERE id NOT IN (SELECT id FROM telemetry ORDER BY id DESC LIMIT ?)`,
      [HISTORY_LIMIT],
      (err) => (err ? reject(err) : resolve())
    );
  });
}

const mqttClient = mqtt.connect(MQTT_URL, {
  reconnectPeriod: 2000,
});

mqttClient.on("connect", () => {
  console.log(`[MQTT] Connecte: ${MQTT_URL}`);
  mqttClient.subscribe(MQTT_TOPIC, (err) => {
    if (err) {
      console.error("[MQTT] Erreur subscribe:", err.message);
    } else {
      console.log(`[MQTT] Abonne: ${MQTT_TOPIC}`);
    }
  });
});

mqttClient.on("error", (err) => {
  console.error("[MQTT] Erreur:", err.message);
});

mqttClient.on("message", (_topic, buffer) => {
  let payload;
  try {
    payload = JSON.parse(buffer.toString("utf8"));
  } catch (_e) {
    return;
  }
  const row = sanitizeTelemetry(payload);
  if (!row) return;

  state.last = row;
  state.history.push(row);
  if (state.history.length > HISTORY_LIMIT) {
    state.history.splice(0, state.history.length - HISTORY_LIMIT);
  }
  saveTelemetry(row).catch((err) => {
    console.error("[DB] Erreur sauvegarde:", err.message);
  });

  io.emit("telemetry", row);
});

io.on("connection", (socket) => {
  socket.emit("bootstrap", {
    last: state.last,
    history: state.history,
  });
});

initDb()
  .then(() => loadHistory())
  .then((history) => {
    state.history = history;
    state.last = history.length > 0 ? history[history.length - 1] : null;
    console.log(`[DB] Backend=${usePostgres ? "postgres" : "sqlite"} historique=${history.length}`);
  })
  .catch((err) => {
    console.error("[DB] Initialisation erreur:", err.message);
  });

server.listen(HTTPS_PORT, () => {
  if (httpsEnabled) {
    console.log(`[HTTPS] Dashboard: https://127.0.0.1:${HTTPS_PORT}`);
  } else {
    console.log(`[HTTP] Dashboard: http://127.0.0.1:${HTTPS_PORT}`);
    console.warn(`[TLS] PFX manquant: ${TLS_PFX_PATH}`);
  }
});

if (httpsEnabled) {
  const redirectApp = express();
  redirectApp.use((req, res) => {
    const host = (req.headers.host || "").split(":")[0] || "127.0.0.1";
    res.redirect(301, `https://${host}:${HTTPS_PORT}${req.originalUrl}`);
  });
  http.createServer(redirectApp).listen(REDIRECT_HTTP_PORT, () => {
    console.log(`[HTTP] Redirect: http://127.0.0.1:${REDIRECT_HTTP_PORT} -> https://127.0.0.1:${HTTPS_PORT}`);
  });
}

