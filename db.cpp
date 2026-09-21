#pragma once
#include <sqlite3.h>

namespace sqlite {
    static_assert(sizeof(sqlite3_int64) == 8, "SQLite int64 size mismatch");

    class Database {
    public:
        sqlite3* handle{nullptr};
        b32 transaction_open{0};
        u64 rows_written{0};
        u64 commit_count{0};

        b32 Open(const c8* path) {
            if (sqlite3_open(path, &handle) != SQLITE_OK) return 0;
            Exec("PRAGMA journal_mode=WAL;");
            Exec("PRAGMA synchronous=NORMAL;");
            Exec("PRAGMA foreign_keys=ON;");
            return 1;
        }

        void Close() {
            if (!handle) return;
            Commit();
            sqlite3_close(handle);
            handle = nullptr;
        }

        b32 Exec(const c8* sql) {
            c8* error_message = nullptr;
            if (sqlite3_exec(handle, sql, nullptr, nullptr, &error_message) != SQLITE_OK) {
                sdl::log("SQLite Exec Error: %s\nQuery: %s", error_message ? error_message : "unknown", sql);
                sqlite3_free(error_message);
                return 0;
            }
            return 1;
        }

        void Begin() { if (!transaction_open) { Exec("BEGIN"); transaction_open = 1; } }
        void Commit() { if (transaction_open) { Exec("COMMIT"); transaction_open = 0; commit_count++; } }
    };

    class Statement {
    public:
        sqlite3_stmt* handle{nullptr};

        b32 Prepare(Database* db, const c8* sql) {
            if (sqlite3_prepare_v2(db->handle, sql, -1, &handle, nullptr) != SQLITE_OK) {
                sdl::log("SQLite Prepare Error: %s\nQuery: %s", sqlite3_errmsg(db->handle), sql);
                return 0;
            }
            return 1;
        }

        void Finalize() {
            if (handle) { sqlite3_finalize(handle); handle = nullptr; }
        }

        inline void BindInt64(i32 index, i64 value) { sqlite3_bind_int64(handle, index, value); }
        inline void BindDouble(i32 index, f64 value) { sqlite3_bind_double(handle, index, value); }
        inline void Step() { sqlite3_step(handle); }
        inline void Reset() { sqlite3_reset(handle); }
    };
}

constexpr u32 EventKind_OrderAssigned   = 1;
constexpr u32 EventKind_OrderDelivered  = 2;
constexpr u32 EventKind_Congestion      = 3;

constexpr u64 EventCapacity = 1 << 16;
constexpr u64 DbFlushIntervalNs = 1000000000ull;

class Event {
public:
    u32 kind;
    u32 agent_id, node_a, node_b, extra;
    u64 order_id, tick, run_ns;
};

struct EventBuffer {
    Event* data;
    u64 count;
    u64 capacity;
    void Init(Arena* arena, u64 cap) { data = PushArray<Event>(arena, cap); capacity = cap; count = 0; }
    void Clear() { count = 0; }
    Event* Push() { if (count >= capacity) return nullptr; return &data[count++]; }
};

class EventLogger {
public:
    sqlite::Database* database;
    EventBuffer events;
    u64 run_start_ns;
    u64 total_notified;

    sqlite::Statement insert_node;
    sqlite::Statement insert_edge;
    sqlite::Statement insert_agent;
    sqlite::Statement insert_order;
    sqlite::Statement complete_order;
    sqlite::Statement insert_event;

    b32 Init(Arena* arena, sqlite::Database* target_database, u64 start_ns) {
        database = target_database;
        run_start_ns = start_ns;
        events.Init(arena, EventCapacity);

        b32 schema_ok = database->Exec(
            "CREATE TABLE run_meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);"
            "CREATE TABLE nodes(node_id INTEGER PRIMARY KEY, x REAL NOT NULL, y REAL NOT NULL, is_hub INTEGER NOT NULL, is_delivery INTEGER NOT NULL, color_r INTEGER NOT NULL, color_g INTEGER NOT NULL, color_b INTEGER NOT NULL);"
            "CREATE TABLE edges(edge_id INTEGER PRIMARY KEY, src_node INTEGER NOT NULL REFERENCES nodes(node_id), dst_node INTEGER NOT NULL REFERENCES nodes(node_id), weight REAL NOT NULL, road_class INTEGER NOT NULL);"
            "CREATE TABLE agents(agent_id INTEGER PRIMARY KEY, home_hub INTEGER NOT NULL REFERENCES nodes(node_id));"
            "CREATE TABLE orders(order_id INTEGER PRIMARY KEY, agent_id INTEGER NOT NULL REFERENCES agents(agent_id), from_node INTEGER NOT NULL REFERENCES nodes(node_id), to_node INTEGER NOT NULL REFERENCES nodes(node_id), path_len INTEGER NOT NULL, assigned_tick INTEGER NOT NULL, assigned_run_ns INTEGER NOT NULL, delivered_tick INTEGER, delivered_run_ns INTEGER);"
            "CREATE TABLE events(event_id INTEGER PRIMARY KEY AUTOINCREMENT, kind INTEGER NOT NULL, tick INTEGER NOT NULL, run_ns INTEGER NOT NULL, agent_id INTEGER NOT NULL REFERENCES agents(agent_id), node_a INTEGER NOT NULL, node_b INTEGER NOT NULL, extra INTEGER NOT NULL, order_id INTEGER NOT NULL);"
            "CREATE INDEX idx_edges_src ON edges(src_node);"
            "CREATE INDEX idx_orders_agent ON orders(agent_id);"
            "CREATE INDEX idx_orders_delivered ON orders(delivered_tick);"
            "CREATE INDEX idx_events_kind_cell ON events(kind, node_a);"
        );
        if (!schema_ok) return 0;

        b32 prepared_ok =
            insert_node.Prepare(database, "INSERT INTO nodes(node_id,x,y,is_hub,is_delivery,color_r,color_g,color_b) VALUES(?,?,?,?,?,?,?,?)") &&
            insert_edge.Prepare(database, "INSERT INTO edges(edge_id,src_node,dst_node,weight,road_class) VALUES(?,?,?,?,?)") &&
            insert_agent.Prepare(database, "INSERT INTO agents(agent_id,home_hub) VALUES(?,?)") &&
            insert_order.Prepare(database, "INSERT INTO orders(order_id,agent_id,from_node,to_node,path_len,assigned_tick,assigned_run_ns) VALUES(?,?,?,?,?,?,?)") &&
            complete_order.Prepare(database, "UPDATE orders SET delivered_tick=?, delivered_run_ns=? WHERE order_id=?") &&
            insert_event.Prepare(database, "INSERT INTO events(kind,tick,run_ns,agent_id,node_a,node_b,extra,order_id) VALUES(?,?,?,?,?,?,?,?)");
        
        if (!prepared_ok) return 0;

        database->Begin();
        return 1;
    }

    void Flush() {
        for (u64 event_index = 0; event_index < events.count; ++event_index) {
            Event* event = &events.data[event_index];
            switch (event->kind) {
                case EventKind_OrderAssigned: {
                    insert_order.BindInt64(1, (sqlite3_int64)event->order_id);
                    insert_order.BindInt64(2, event->agent_id);
                    insert_order.BindInt64(3, event->node_a);
                    insert_order.BindInt64(4, event->node_b);
                    insert_order.BindInt64(5, event->extra);
                    insert_order.BindInt64(6, (sqlite3_int64)event->tick);
                    insert_order.BindInt64(7, (sqlite3_int64)event->run_ns);
                    insert_order.Step(); insert_order.Reset();
                    database->rows_written++;
                } break;
                case EventKind_OrderDelivered: {
                    complete_order.BindInt64(1, (sqlite3_int64)event->tick);
                    complete_order.BindInt64(2, (sqlite3_int64)event->run_ns);
                    complete_order.BindInt64(3, (sqlite3_int64)event->order_id);
                    complete_order.Step(); complete_order.Reset();
                } break;
                default: break;
            }
            insert_event.BindInt64(1, event->kind);
            insert_event.BindInt64(2, (sqlite3_int64)event->tick);
            insert_event.BindInt64(3, (sqlite3_int64)event->run_ns);
            insert_event.BindInt64(4, event->agent_id);
            insert_event.BindInt64(5, event->node_a);
            insert_event.BindInt64(6, event->node_b);
            insert_event.BindInt64(7, event->extra);
            insert_event.BindInt64(8, (sqlite3_int64)event->order_id);
            insert_event.Step(); insert_event.Reset();
            database->rows_written++;
        }
        events.Clear();
    }

    void Notify(u32 kind, u64 tick, u32 agent_id, u32 node_a, u32 node_b, u32 extra, u64 order_id) {
        if (events.count >= events.capacity) Flush();
        Event* event = events.Push();
        if (event) {
            event->kind = kind; event->tick = tick; event->run_ns = sdl::now_ns() - run_start_ns;
            event->agent_id = agent_id; event->node_a = node_a; event->node_b = node_b;
            event->extra = extra; event->order_id = order_id;
            total_notified++;
        }
    }

    void Close() {
        insert_node.Finalize();
        insert_edge.Finalize();
        insert_agent.Finalize();
        insert_order.Finalize();
        complete_order.Finalize();
        insert_event.Finalize();
    }
};