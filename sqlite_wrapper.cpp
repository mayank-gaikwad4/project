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

        b32 exec(const c8* sql) {
            c8* error_message = nullptr;
            if (sqlite3_exec(handle, sql, nullptr, nullptr, &error_message) != SQLITE_OK) {
                sdl::log("sqlite error: %s", error_message ? error_message : "unknown");
                sqlite3_free(error_message);
                return 0;
            }
            return 1;
        }

        b32 prepare(sqlite3_stmt** statement, const c8* sql) {
            if (sqlite3_prepare_v2(handle, sql, -1, statement, nullptr) != SQLITE_OK) {
                sdl::log("sqlite prepare failed: %s", sqlite3_errmsg(handle));
                return 0;
            }
            return 1;
        }

        void begin() { if (!transaction_open) { exec("BEGIN"); transaction_open = 1; } }
        void commit() { if (transaction_open) { exec("COMMIT"); transaction_open = 0; commit_count++; } }

        b32 open(const c8* path) {
            if (sqlite3_open(path, &handle) != SQLITE_OK) return 0;
            exec("PRAGMA journal_mode=WAL;");
            exec("PRAGMA synchronous=NORMAL;");
            exec("PRAGMA foreign_keys=ON;");
            return 1;
        }

        void close() {
            if (!handle) return;
            commit();
            sqlite3_close(handle);
            handle = nullptr;
        }
    };
}