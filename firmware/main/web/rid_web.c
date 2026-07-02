#include "web/rid_web.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "opendroneid.h"
#include "protocol/rid_state.h"
#include "transport/rid_mavlink.h"
#include "web/rid_firmware_check.h"

static const char *TAG = "rid_web";
static httpd_handle_t s_server;

typedef struct {
    const esp_partition_t *part;
    esp_ota_handle_t handle;
    bool started;
    esp_err_t err;
    size_t written;
} ota_upload_t;

typedef struct {
    int value;
    const char *name;
} enum_name_t;

#define ARRAY_LEN(_a) (sizeof(_a) / sizeof((_a)[0]))

static const enum_name_t ENUM_UATYPE[] = {
    { ODID_UATYPE_NONE, "NONE" },
    { ODID_UATYPE_AEROPLANE, "AEROPLANE" },
    { ODID_UATYPE_HELICOPTER_OR_MULTIROTOR, "HELICOPTER_OR_MULTIROTOR" },
    { ODID_UATYPE_GYROPLANE, "GYROPLANE" },
    { ODID_UATYPE_HYBRID_LIFT, "HYBRID_LIFT" },
    { ODID_UATYPE_ORNITHOPTER, "ORNITHOPTER" },
    { ODID_UATYPE_GLIDER, "GLIDER" },
    { ODID_UATYPE_KITE, "KITE" },
    { ODID_UATYPE_FREE_BALLOON, "FREE_BALLOON" },
    { ODID_UATYPE_CAPTIVE_BALLOON, "CAPTIVE_BALLOON" },
    { ODID_UATYPE_AIRSHIP, "AIRSHIP" },
    { ODID_UATYPE_FREE_FALL_PARACHUTE, "FREE_FALL_PARACHUTE" },
    { ODID_UATYPE_ROCKET, "ROCKET" },
    { ODID_UATYPE_TETHERED_POWERED_AIRCRAFT, "TETHERED_POWERED_AIRCRAFT" },
    { ODID_UATYPE_GROUND_OBSTACLE, "GROUND_OBSTACLE" },
    { ODID_UATYPE_OTHER, "OTHER" },
};

static const enum_name_t ENUM_IDTYPE[] = {
    { ODID_IDTYPE_NONE, "NONE" },
    { ODID_IDTYPE_SERIAL_NUMBER, "SERIAL_NUMBER" },
    { ODID_IDTYPE_CAA_REGISTRATION_ID, "CAA_REGISTRATION_ID" },
    { ODID_IDTYPE_UTM_ASSIGNED_UUID, "UTM_ASSIGNED_UUID" },
    { ODID_IDTYPE_SPECIFIC_SESSION_ID, "SPECIFIC_SESSION_ID" },
};

static const enum_name_t ENUM_LOCTYPE[] = {
    { ODID_OPERATOR_LOCATION_TYPE_TAKEOFF, "TAKEOFF" },
    { ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS, "LIVE_GNSS" },
    { ODID_OPERATOR_LOCATION_TYPE_FIXED, "FIXED" },
};

static const enum_name_t ENUM_CLASSIF[] = {
    { ODID_CLASSIFICATION_TYPE_UNDECLARED, "UNDECLARED" },
    { ODID_CLASSIFICATION_TYPE_EU, "EU" },
};

static const enum_name_t ENUM_STATUS[] = {
    { ODID_STATUS_UNDECLARED, "UNDECLARED" },
    { ODID_STATUS_GROUND, "GROUND" },
    { ODID_STATUS_AIRBORNE, "AIRBORNE" },
    { ODID_STATUS_EMERGENCY, "EMERGENCY" },
    { ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE, "REMOTE_ID_SYSTEM_FAILURE" },
};

static const enum_name_t ENUM_HEIGHT[] = {
    { ODID_HEIGHT_REF_OVER_TAKEOFF, "OVER_TAKEOFF" },
    { ODID_HEIGHT_REF_OVER_GROUND, "OVER_GROUND" },
};

static const enum_name_t ENUM_HACC[] = {
    { ODID_HOR_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_HOR_ACC_10NM, "10 nm" },
    { ODID_HOR_ACC_4NM, "4 nm" },
    { ODID_HOR_ACC_2NM, "2 nm" },
    { ODID_HOR_ACC_1NM, "1 nm" },
    { ODID_HOR_ACC_0_5NM, "0.5 nm" },
    { ODID_HOR_ACC_0_3NM, "0.3 nm" },
    { ODID_HOR_ACC_0_1NM, "0.1 nm" },
    { ODID_HOR_ACC_0_05NM, "0.05 nm" },
    { ODID_HOR_ACC_30_METER, "30 m" },
    { ODID_HOR_ACC_10_METER, "10 m" },
    { ODID_HOR_ACC_3_METER, "3 m" },
    { ODID_HOR_ACC_1_METER, "1 m" },
};

static const enum_name_t ENUM_VACC[] = {
    { ODID_VER_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_VER_ACC_150_METER, "150 m" },
    { ODID_VER_ACC_45_METER, "45 m" },
    { ODID_VER_ACC_25_METER, "25 m" },
    { ODID_VER_ACC_10_METER, "10 m" },
    { ODID_VER_ACC_3_METER, "3 m" },
    { ODID_VER_ACC_1_METER, "1 m" },
};

static const enum_name_t ENUM_SACC[] = {
    { ODID_SPEED_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_SPEED_ACC_10_METERS_PER_SECOND, "10 m/s" },
    { ODID_SPEED_ACC_3_METERS_PER_SECOND, "3 m/s" },
    { ODID_SPEED_ACC_1_METERS_PER_SECOND, "1 m/s" },
    { ODID_SPEED_ACC_0_3_METERS_PER_SECOND, "0.3 m/s" },
};

static const enum_name_t ENUM_DESCTYPE[] = {
    { ODID_DESC_TYPE_TEXT, "TEXT" },
    { ODID_DESC_TYPE_EMERGENCY, "EMERGENCY" },
    { ODID_DESC_TYPE_EXTENDED_STATUS, "EXTENDED_STATUS" },
};

static const enum_name_t ENUM_CLASSEU[] = {
    { ODID_CLASS_EU_UNDECLARED, "UNDECLARED" },
    { ODID_CLASS_EU_CLASS_0, "CLASS_0" },
    { ODID_CLASS_EU_CLASS_1, "CLASS_1" },
    { ODID_CLASS_EU_CLASS_2, "CLASS_2" },
    { ODID_CLASS_EU_CLASS_3, "CLASS_3" },
    { ODID_CLASS_EU_CLASS_4, "CLASS_4" },
    { ODID_CLASS_EU_CLASS_5, "CLASS_5" },
    { ODID_CLASS_EU_CLASS_6, "CLASS_6" },
};

static const enum_name_t ENUM_CATEU[] = {
    { ODID_CATEGORY_EU_UNDECLARED, "UNDECLARED" },
    { ODID_CATEGORY_EU_OPEN, "OPEN" },
    { ODID_CATEGORY_EU_SPECIFIC, "SPECIFIC" },
    { ODID_CATEGORY_EU_CERTIFIED, "CERTIFIED" },
};

static const enum_name_t ENUM_TSACC[] = {
    { ODID_TIME_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_TIME_ACC_0_1_SECOND, "0.1 s" },
    { ODID_TIME_ACC_0_2_SECOND, "0.2 s" },
    { ODID_TIME_ACC_0_3_SECOND, "0.3 s" },
    { ODID_TIME_ACC_0_4_SECOND, "0.4 s" },
    { ODID_TIME_ACC_0_5_SECOND, "0.5 s" },
    { ODID_TIME_ACC_0_6_SECOND, "0.6 s" },
    { ODID_TIME_ACC_0_7_SECOND, "0.7 s" },
    { ODID_TIME_ACC_0_8_SECOND, "0.8 s" },
    { ODID_TIME_ACC_0_9_SECOND, "0.9 s" },
    { ODID_TIME_ACC_1_0_SECOND, "1.0 s" },
    { ODID_TIME_ACC_1_1_SECOND, "1.1 s" },
    { ODID_TIME_ACC_1_2_SECOND, "1.2 s" },
    { ODID_TIME_ACC_1_3_SECOND, "1.3 s" },
    { ODID_TIME_ACC_1_4_SECOND, "1.4 s" },
    { ODID_TIME_ACC_1_5_SECOND, "1.5 s" },
};

static bool appendf(char **cursor, size_t *remaining, const char *fmt, ...) {
    if (!cursor || !*cursor || !remaining || *remaining == 0) {
        return false;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(*cursor, *remaining, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return false;
    }
    if ((size_t)n >= *remaining) {
        *cursor += *remaining - 1;
        *remaining = 1;
        return false;
    }
    *cursor += n;
    *remaining -= (size_t)n;
    return true;
}

static void json_value_escaped(char **cursor, size_t *remaining, const char *value) {
    const unsigned char *s = (const unsigned char *)(value ? value : "");
    while (*s != 0 && remaining && *remaining > 1) {
        if (*s == '"' || *s == '\\') {
            appendf(cursor, remaining, "\\%c", *s);
        } else if (*s >= 0x20) {
            appendf(cursor, remaining, "%c", *s);
        }
        s++;
    }
}

static void json_add_string(char **cursor,
                            size_t *remaining,
                            bool *first,
                            const char *key,
                            const char *value) {
    appendf(cursor, remaining, "%s\"%s\":\"", *first ? "" : ",", key);
    json_value_escaped(cursor, remaining, value);
    appendf(cursor, remaining, "\"");
    *first = false;
}

static void json_add_fmt(char **cursor,
                         size_t *remaining,
                         bool *first,
                         const char *key,
                         const char *fmt,
                         ...) {
    char value[64] = {0};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(value, sizeof(value), fmt, ap);
    va_end(ap);
    json_add_string(cursor, remaining, first, key, value);
}

static const char *enum_name(const enum_name_t *map,
                             size_t map_len,
                             int value,
                             char fallback[16]) {
    for (size_t i = 0; i < map_len; i++) {
        if (map[i].value == value) {
            return map[i].name;
        }
    }
    snprintf(fallback, 16, "%d", value);
    return fallback;
}

static void json_add_enum(char **cursor,
                          size_t *remaining,
                          bool *first,
                          const char *key,
                          const enum_name_t *map,
                          size_t map_len,
                          int value) {
    char fallback[16] = {0};
    json_add_string(cursor, remaining, first, key, enum_name(map, map_len, value, fallback));
}

static const char *latlon_string(double lat, double lon, bool latitude, char out[24]) {
    if (lat == 0.0 && lon == 0.0) {
        return "UNKNOWN";
    }
    snprintf(out, 24, "%.8f", latitude ? lat : lon);
    return out;
}

static const char *alt_string(float alt, char out[24]) {
    if (alt <= -1000.0f) {
        return "UNKNOWN";
    }
    snprintf(out, 24, "%.2f", (double)alt);
    return out;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    static const char html[] =
        "<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>SeryRemoteID</title><style>"
        ":root{--bg:#f5f7fa;--panel:#fff;--ink:#18202a;--muted:#687386;--line:#d9e0e8;--ok:#16784a;--bad:#b42318;--warn:#9a6700;--chip:#edf1f5}"
        "*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:\"Segoe UI\",Tahoma,sans-serif;letter-spacing:0}"
        ".wrap{max-width:1120px;margin:0 auto;padding:20px}.top{display:flex;align-items:flex-start;justify-content:space-between;gap:16px;margin-bottom:16px}"
        "h1{margin:0;font-size:28px;line-height:1.1}h2{margin:0 0 12px;font-size:15px}.sub{margin-top:6px;color:var(--muted);font-size:13px}"
        ".badge{border:1px solid var(--line);border-radius:6px;background:var(--panel);padding:10px 12px;min-width:118px;text-align:center;font-weight:700}.badge.ok{color:var(--ok);border-color:#8fd5b2}.badge.bad{color:var(--bad);border-color:#efb1aa}"
        ".summary{display:grid;grid-template-columns:1.2fr 2fr;gap:12px;margin-bottom:12px}.hero,.panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:14px}"
        ".hero strong{display:block;font-size:20px;margin-top:4px}.chips{display:flex;flex-wrap:wrap;gap:6px}.chip{background:var(--chip);border:1px solid var(--line);border-radius:999px;padding:4px 8px;font-size:12px;color:var(--muted)}.chip.bad{color:var(--bad);background:#fff1f0;border-color:#f3bbb5}"
        ".grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.wide{grid-column:span 2}.kv{display:grid;grid-template-columns:minmax(96px,44%) minmax(0,1fr);gap:10px;padding:7px 0;border-top:1px solid #eef2f6}.kv:first-child{border-top:0}.kv span{color:var(--muted);font-size:12px}.kv b{font-size:13px;overflow-wrap:anywhere;font-weight:650}"
        ".ota{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.ota input{max-width:100%;font-size:13px}.btn{border:1px solid #1f2937;background:#1f2937;color:#fff;border-radius:6px;padding:8px 12px;font-weight:700;cursor:pointer}.btn:disabled{opacity:.55;cursor:not-allowed}"
        "#otaStatus{color:var(--muted);font-size:12px}.raw{white-space:pre-wrap;background:#111827;color:#d1fae5;border-radius:6px;padding:12px;overflow:auto;max-height:340px;font:12px/1.45 ui-monospace,Consolas,monospace}"
        "details{margin-top:12px}summary{cursor:pointer;color:var(--muted);font-size:13px}@media(max-width:760px){.wrap{padding:14px}.top,.summary{display:block}.badge{margin-top:12px}.grid{grid-template-columns:1fr}.wide{grid-column:auto}}"
        "</style></head><body><main class=\"wrap\">"
        "<header class=\"top\"><div><h1>SeryRemoteID</h1><div class=\"sub\"><span id=\"app\">sery_remoteid</span> | <span id=\"version\">-</span> | <span id=\"uptime\">--:--:--</span></div></div><div id=\"badge\" class=\"badge bad\">WAITING</div></header>"
        "<section class=\"summary\"><div class=\"hero\"><span>RemoteID arm state</span><strong id=\"armText\">Waiting for data</strong></div><div class=\"panel\"><h2>Missing Inputs</h2><div id=\"chips\" class=\"chips\"></div></div></section>"
        "<section class=\"grid\">"
        "<div class=\"panel\"><h2>Device</h2><div id=\"deviceRows\"></div></div>"
        "<div class=\"panel\"><h2>Basic ID</h2><div id=\"basicRows\"></div></div>"
        "<div class=\"panel\"><h2>Location</h2><div id=\"locationRows\"></div></div>"
        "<div class=\"panel\"><h2>Operator</h2><div id=\"operatorRows\"></div></div>"
        "<div class=\"panel\"><h2>System</h2><div id=\"systemRows\"></div></div>"
        "<div class=\"panel\"><h2>OTA</h2><div class=\"ota\"><input id=\"fw\" type=\"file\" accept=\".bin,application/octet-stream\"><button id=\"upload\" class=\"btn\" type=\"button\">Update</button><span id=\"otaStatus\"></span></div></div>"
        "</section><details><summary>Raw JSON</summary><pre id=\"raw\" class=\"raw\">Loading...</pre></details>"
        "</main><script>"
        "const $=id=>document.getElementById(id);"
        "const clean=v=>(v===undefined||v===null||v==='')?'-':String(v);"
        "function rows(id,defs,d){const el=$(id);el.replaceChildren();for(const [label,key] of defs){const r=document.createElement('div');r.className='kv';const s=document.createElement('span');s.textContent=label;const b=document.createElement('b');b.textContent=clean(d[key]);r.append(s,b);el.append(r);}}"
        "function chips(reason,ok){const el=$('chips');el.replaceChildren();const parts=String(reason||'').trim().split(/\\s+/).filter(Boolean);if(parts.length===0){parts.push(ok?'Complete':'Waiting');}for(const p of parts){const c=document.createElement('span');c.className='chip'+(ok?'':' bad');c.textContent=p;el.append(c);}}"
        "function render(d){const ok=d.arm_ok===true||d.arm_ok==='true';$('badge').className='badge '+(ok?'ok':'bad');$('badge').textContent=ok?'READY':'BLOCKED';$('armText').textContent=ok?'Good to arm':'Not ready';$('app').textContent=clean(d.app);$('version').textContent='v'+clean(d.version);$('uptime').textContent='uptime '+clean(d['STATUS:UPTIME']);chips(d.reason,ok);rows('deviceRows',[['WiFi SSID','wifi_ssid'],['WiFi channel','wifi_channel'],['MAVLink sysid','mavlink_sysid'],['Board ID','STATUS:BOARD_ID'],['Free heap','STATUS:FREEMEM']],d);rows('basicRows',[['UA type','BASICID:UAType'],['ID type','BASICID:IDType'],['UAS ID','BASICID:UASID'],['UA type 2','BASICID:UAType2'],['ID type 2','BASICID:IDType2'],['UAS ID 2','BASICID:UASID2']],d);rows('locationRows',[['Status','LOCATION:Status'],['Reason','LOCATION:StatusReason'],['Latitude','LOCATION:Latitude'],['Longitude','LOCATION:Longitude'],['Alt geo','LOCATION:AltitudeGeo'],['Alt baro','LOCATION:AltitudeBaro'],['Height','LOCATION:Height'],['Speed H','LOCATION:SpeedHorizontal'],['Speed V','LOCATION:SpeedVertical'],['Direction','LOCATION:Direction'],['Time','LOCATION:TimeStamp']],d);rows('operatorRows',[['Operator ID type','OPERATORID:IDType'],['Operator ID','OPERATORID:ID'],['Self ID type','SELFID:DescType'],['Self ID','SELFID:Desc']],d);rows('systemRows',[['Operator loc type','SYSTEM:OperatorLocationType'],['Operator lat','SYSTEM:OperatorLatitude'],['Operator lon','SYSTEM:OperatorLongitude'],['Area count','SYSTEM:AreaCount'],['Area radius','SYSTEM:AreaRadius'],['Ceiling','SYSTEM:AreaCeiling'],['Floor','SYSTEM:AreaFloor'],['EU category','SYSTEM:CategoryEU'],['EU class','SYSTEM:ClassEU']],d);$('raw').textContent=JSON.stringify(d,null,2);}"
        "async function load(){try{const r=await fetch('/ajax/status.json',{cache:'no-store'});render(await r.json());}catch(e){$('badge').className='badge bad';$('badge').textContent='OFFLINE';$('raw').textContent=String(e);}}"
        "async function flash(){const f=$('fw').files[0];const st=$('otaStatus');if(!f){st.textContent='No file selected';return;}const btn=$('upload');btn.disabled=true;st.textContent='Uploading...';try{const r=await fetch('/update',{method:'POST',headers:{'Content-Type':'application/octet-stream'},body:f});const t=await r.text();st.textContent=r.ok?'Uploaded, rebooting':(t||'OTA failed');}catch(e){st.textContent='Upload interrupted, device may be rebooting';}finally{btn.disabled=false;}}"
        "$('upload').addEventListener('click',flash);load();setInterval(load,1000);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    char arm_reason[96] = {0};
    char status_reason[128] = {0};
    bool arm_ok = rid_state_arm_status(arm_reason, sizeof(arm_reason));
    ODID_UAS_Data uas;
    (void)rid_state_build_uas_data(&uas, status_reason, sizeof(status_reason));

    const esp_app_desc_t *desc = esp_app_get_description();
    const rid_config_t *cfg = cfg_get();
    char *body = calloc(1, 4096);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
        return ESP_FAIL;
    }

    char *cursor = body;
    size_t remaining = 4096;
    bool first = true;
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t sec = uptime_s % 60U;
    uint32_t min = (uptime_s / 60U) % 60U;
    uint32_t hr = (uptime_s / 3600U) % 24U;
    char tmp[24] = {0};

    appendf(&cursor, &remaining, "{");
    json_add_string(&cursor, &remaining, &first, "app", desc ? desc->project_name : "sery_remoteid");
    json_add_string(&cursor, &remaining, &first, "version", desc ? desc->version : "unknown");
    json_add_string(&cursor, &remaining, &first, "arm_ok", arm_ok ? "true" : "false");
    json_add_string(&cursor, &remaining, &first, "reason", arm_reason);
    json_add_string(&cursor, &remaining, &first, "wifi_ssid", cfg->wifi_ssid);
    json_add_fmt(&cursor, &remaining, &first, "wifi_channel", "%u", (unsigned)cfg->wifi_channel);
    json_add_fmt(&cursor, &remaining, &first, "mavlink_sysid", "%u",
                 (unsigned)rid_mavlink_system_id());

    json_add_string(&cursor, &remaining, &first, "STATUS:VERSION", desc ? desc->version : "unknown");
    json_add_fmt(&cursor, &remaining, &first, "STATUS:BOARD_ID", "%u", SERY_RID_CAN_BOARD_ID);
    json_add_fmt(&cursor, &remaining, &first, "STATUS:UPTIME", "%02lu:%02lu:%02lu",
                 (unsigned long)hr,
                 (unsigned long)min,
                 (unsigned long)sec);
    json_add_fmt(&cursor, &remaining, &first, "STATUS:FREEMEM", "%lu",
                 (unsigned long)esp_get_free_heap_size());

    json_add_enum(&cursor, &remaining, &first, "BASICID:UAType",
                  ENUM_UATYPE, ARRAY_LEN(ENUM_UATYPE), uas.BasicID[0].UAType);
    json_add_enum(&cursor, &remaining, &first, "BASICID:IDType",
                  ENUM_IDTYPE, ARRAY_LEN(ENUM_IDTYPE), uas.BasicID[0].IDType);
    json_add_string(&cursor, &remaining, &first, "BASICID:UASID", uas.BasicID[0].UASID);
    json_add_enum(&cursor, &remaining, &first, "BASICID:UAType2",
                  ENUM_UATYPE, ARRAY_LEN(ENUM_UATYPE), uas.BasicID[1].UAType);
    json_add_enum(&cursor, &remaining, &first, "BASICID:IDType2",
                  ENUM_IDTYPE, ARRAY_LEN(ENUM_IDTYPE), uas.BasicID[1].IDType);
    json_add_string(&cursor, &remaining, &first, "BASICID:UASID2", uas.BasicID[1].UASID);

    json_add_fmt(&cursor, &remaining, &first, "OPERATORID:IDType", "%u",
                 (unsigned)uas.OperatorID.OperatorIdType);
    json_add_string(&cursor, &remaining, &first, "OPERATORID:ID", uas.OperatorID.OperatorId);
    json_add_enum(&cursor, &remaining, &first, "SELFID:DescType",
                  ENUM_DESCTYPE, ARRAY_LEN(ENUM_DESCTYPE), uas.SelfID.DescType);
    json_add_string(&cursor, &remaining, &first, "SELFID:Desc", uas.SelfID.Desc);

    json_add_enum(&cursor, &remaining, &first, "SYSTEM:OperatorLocationType",
                  ENUM_LOCTYPE, ARRAY_LEN(ENUM_LOCTYPE), uas.System.OperatorLocationType);
    json_add_enum(&cursor, &remaining, &first, "SYSTEM:ClassificationType",
                  ENUM_CLASSIF, ARRAY_LEN(ENUM_CLASSIF), uas.System.ClassificationType);
    json_add_string(&cursor, &remaining, &first, "SYSTEM:OperatorLatitude",
                    latlon_string(uas.System.OperatorLatitude, uas.System.OperatorLongitude, true, tmp));
    json_add_string(&cursor, &remaining, &first, "SYSTEM:OperatorLongitude",
                    latlon_string(uas.System.OperatorLatitude, uas.System.OperatorLongitude, false, tmp));
    json_add_fmt(&cursor, &remaining, &first, "SYSTEM:AreaCount", "%u",
                 (unsigned)uas.System.AreaCount);
    json_add_fmt(&cursor, &remaining, &first, "SYSTEM:AreaRadius", "%u",
                 (unsigned)uas.System.AreaRadius);
    json_add_string(&cursor, &remaining, &first, "SYSTEM:AreaCeiling",
                    alt_string(uas.System.AreaCeiling, tmp));
    json_add_string(&cursor, &remaining, &first, "SYSTEM:AreaFloor",
                    alt_string(uas.System.AreaFloor, tmp));
    json_add_enum(&cursor, &remaining, &first, "SYSTEM:CategoryEU",
                  ENUM_CATEU, ARRAY_LEN(ENUM_CATEU), uas.System.CategoryEU);
    json_add_enum(&cursor, &remaining, &first, "SYSTEM:ClassEU",
                  ENUM_CLASSEU, ARRAY_LEN(ENUM_CLASSEU), uas.System.ClassEU);
    json_add_string(&cursor, &remaining, &first, "SYSTEM:OperatorAltitudeGeo",
                    alt_string(uas.System.OperatorAltitudeGeo, tmp));
    json_add_fmt(&cursor, &remaining, &first, "SYSTEM:Timestamp", "%u",
                 (unsigned)uas.System.Timestamp);

    json_add_enum(&cursor, &remaining, &first, "LOCATION:Status",
                  ENUM_STATUS, ARRAY_LEN(ENUM_STATUS), uas.Location.Status);
    json_add_string(&cursor, &remaining, &first, "LOCATION:StatusReason",
                    status_reason[0] ? status_reason : arm_reason);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:Direction", "%.2f",
                 (double)uas.Location.Direction);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:SpeedHorizontal", "%.2f",
                 (double)uas.Location.SpeedHorizontal);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:SpeedVertical", "%.2f",
                 (double)uas.Location.SpeedVertical);
    json_add_string(&cursor, &remaining, &first, "LOCATION:Latitude",
                    latlon_string(uas.Location.Latitude, uas.Location.Longitude, true, tmp));
    json_add_string(&cursor, &remaining, &first, "LOCATION:Longitude",
                    latlon_string(uas.Location.Latitude, uas.Location.Longitude, false, tmp));
    json_add_string(&cursor, &remaining, &first, "LOCATION:AltitudeBaro",
                    alt_string(uas.Location.AltitudeBaro, tmp));
    json_add_string(&cursor, &remaining, &first, "LOCATION:AltitudeGeo",
                    alt_string(uas.Location.AltitudeGeo, tmp));
    json_add_enum(&cursor, &remaining, &first, "LOCATION:HeightType",
                  ENUM_HEIGHT, ARRAY_LEN(ENUM_HEIGHT), uas.Location.HeightType);
    json_add_string(&cursor, &remaining, &first, "LOCATION:Height",
                    alt_string(uas.Location.Height, tmp));
    json_add_enum(&cursor, &remaining, &first, "LOCATION:HorizAccuracy",
                  ENUM_HACC, ARRAY_LEN(ENUM_HACC), uas.Location.HorizAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:VertAccuracy",
                  ENUM_VACC, ARRAY_LEN(ENUM_VACC), uas.Location.VertAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:BaroAccuracy",
                  ENUM_VACC, ARRAY_LEN(ENUM_VACC), uas.Location.BaroAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:SpeedAccuracy",
                  ENUM_SACC, ARRAY_LEN(ENUM_SACC), uas.Location.SpeedAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:TSAccuracy",
                  ENUM_TSACC, ARRAY_LEN(ENUM_TSACC), uas.Location.TSAccuracy);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:TimeStamp", "%.2f",
                 (double)uas.Location.TimeStamp);
    appendf(&cursor, &remaining, "}");

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(body);
    return ret;
}

static esp_err_t favicon_get_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t update_post_handler(httpd_req_t *req) {
    ota_upload_t ota = {
        .part = esp_ota_get_next_update_partition(NULL),
        .handle = 0,
        .started = false,
        .err = ESP_OK,
        .written = 0,
    };
    if (!ota.part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA partition");
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(esp_ota_begin(ota.part, OTA_SIZE_UNKNOWN, &ota.handle),
                        TAG,
                        "esp_ota_begin failed");
    ota.started = true;

    char buffer[1024];
    int remaining = req->content_len;
    while (remaining > 0) {
        int recv = httpd_req_recv(req, buffer, remaining > (int)sizeof(buffer) ? sizeof(buffer) : remaining);
        if (recv <= 0) {
            ota.err = ESP_FAIL;
            break;
        }
        ota.err = esp_ota_write(ota.handle, buffer, recv);
        if (ota.err != ESP_OK) {
            break;
        }
        ota.written += (size_t)recv;
        remaining -= recv;
    }

    if (ota.err == ESP_OK) {
        ota.err = esp_ota_end(ota.handle);
    } else if (ota.started) {
        esp_ota_abort(ota.handle);
    }
    if (ota.err == ESP_OK && !rid_firmware_check_ota_partition(ota.part)) {
        ota.err = ESP_ERR_INVALID_CRC;
    }
    if (ota.err == ESP_OK) {
        ota.err = esp_ota_set_boot_partition(ota.part);
    }
    if (ota.err != ESP_OK) {
        ESP_LOGE(TAG, "OTA failed after %u bytes: %s", (unsigned)ota.written, esp_err_to_name(ota.err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK");
    ESP_LOGW(TAG, "OTA update written, restarting");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

esp_err_t rid_web_start(void) {
    if (!cfg_get()->webserver_enable) {
        ESP_LOGI(TAG, "web server disabled");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "httpd_start failed");

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    const httpd_uri_t status = {
        .uri = "/ajax/status.json",
        .method = HTTP_GET,
        .handler = status_get_handler,
    };
    const httpd_uri_t update = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = update_post_handler,
    };
    const httpd_uri_t favicon = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = favicon_get_handler,
    };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &status);
    httpd_register_uri_handler(s_server, &update);
    httpd_register_uri_handler(s_server, &favicon);
    ESP_LOGI(TAG, "web server started");
    return ESP_OK;
}
