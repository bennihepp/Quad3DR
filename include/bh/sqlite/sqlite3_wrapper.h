//==================================================
// sqlite3_wrapper.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 03.04.17
//==================================================
#pragma once

#include "../common.h"
#include <sqlite3.h>

namespace bh {

class SQLite3 {
public:
  using size_t = std::size_t;
  using string = std::string;

  class Error : public bh::Error {
  public:
    explicit Error(const std::string& what)
        : bh::Error(what) {}

    explicit Error(const char* what)
        : bh::Error(what) {}
  };

  enum OpenMode {
    READONLY = 1,
    READWRITE = 2,
    READWRITE_OPEN_CREATE = 3,
  };

  class RowResult {
  public:
    size_t numColumns() const;

    string getString(size_t col) const;

    int getInt(size_t col) const;

    int64_t getInt64(size_t col) const;

    double getDouble(size_t col) const;

    std::vector<uint8_t> getRawData(size_t col) const;

    template <typename T>
    std::vector<T> getData(size_t col) const;

    template <typename T>
    std::vector<T> getData(size_t col, const int count) const;

  private:
    friend class SQLite3;

    RowResult(sqlite3_stmt* stmt);

    sqlite3_stmt* stmt_;
  };

  class Statement {
  public:
    ~Statement();

    void bindNull(const int index);

    void bindValue(const int index, const int value);

    void bindValue(const int index, const size_t value);

    void bindValue64(const int index, const size_t value);

    void bindValue(const int index, const double value);

    template <typename T>
    void bindData(const int index, const std::vector<T>& data);

    void finish();

  private:
    friend class SQLite3;

    Statement(SQLite3* sqlite_db, sqlite3_stmt* stmt);

    SQLite3* sqlite_db_;
    sqlite3_stmt* stmt_;
  };

  SQLite3(const string& filename, const OpenMode mode);

  ~SQLite3();

  Statement prepare(const string& query);

  void executeWithoutResult(const Statement& statement);

  void executeWithoutResult(const string& query);

  RowResult executeSingle(const Statement& statement);

  template <typename Callback>
  void executeSingle(const string& query, Callback&& callback);

  template <typename Callback>
  void execute(const string& query, Callback&& callback);

private:
  int getFlagsFromOpenMode(const OpenMode mode);

  string getErrorMessage(const int result);

  bool hasErrorOccured(const int result, string* error_msg = nullptr);

  void throwIfError(const int result);

  sqlite3* db_;
};

}

// Implementation
namespace bh {

SQLite3::SQLite3(const string& filename, const OpenMode mode) {
  int flags = getFlagsFromOpenMode(mode);
  const char* zVfs = nullptr;
  int result = sqlite3_open_v2(filename.c_str(), &db_, flags, zVfs);
  throwIfError(result);
}

SQLite3::~SQLite3() {
  sqlite3_close(db_);
}

auto SQLite3::prepare(const string& query) -> Statement {
  sqlite3_stmt* stmt;
  const char** pzTail = nullptr;
  int result = sqlite3_prepare_v2(db_, query.c_str(), query.length(), &stmt, pzTail);
  throwIfError(result);
  return Statement(this, stmt);
}

void SQLite3::executeWithoutResult(const Statement& statement) {
  int result = sqlite3_step(statement.stmt_);
  if (result == SQLITE_ROW) {
    throw Error("Got result when no result was expected");
  }
  else if (result != SQLITE_DONE) {
    throw Error(getErrorMessage(result));
  }
}

auto SQLite3::executeSingle(const Statement& statement) -> RowResult  {
  int result = sqlite3_step(statement.stmt_);
  if (result != SQLITE_ROW && result != SQLITE_DONE) {
    throw Error(getErrorMessage(result));
  }
  else if (result == SQLITE_DONE) {
    throw Error("No result when one result was expected");
  }
  return RowResult(statement.stmt_);
}

void SQLite3::executeWithoutResult(const string& query) {
  sqlite3_stmt* stmt;
  const char** pzTail = nullptr;
  int result = sqlite3_prepare_v2(db_, query.c_str(), query.length(), &stmt, pzTail);
  throwIfError(result);
  result = sqlite3_step(stmt);
  if (result == SQLITE_ROW) {
    throw Error("Got result when no result was expected");
  }
  else if (result != SQLITE_DONE) {
    throw Error(getErrorMessage(result));
  }
  result = sqlite3_finalize(stmt);
  throwIfError(result);
}

template <typename Callback>
void SQLite3::executeSingle(const string& query, Callback&& callback) {
  sqlite3_stmt* stmt;
  const char** pzTail = nullptr;
  int result = sqlite3_prepare_v2(db_, query.c_str(), query.length(), &stmt, pzTail);
  throwIfError(result);
  result = sqlite3_step(stmt);
  if (result != SQLITE_ROW && result != SQLITE_DONE) {
    throw Error(getErrorMessage(result));
  }
  else if (result == SQLITE_DONE) {
    throw Error("No result when one result was expected");
  }
  RowResult row_result(stmt);
  std::forward<Callback>(callback)(row_result);
  result = sqlite3_finalize(stmt);
  throwIfError(result);
}

int SQLite3::getFlagsFromOpenMode(const OpenMode mode) {
  if (mode == READONLY) {
    return SQLITE_OPEN_READONLY;
  }
  else if (mode == READWRITE) {
    return SQLITE_OPEN_READWRITE;
  }
  else if (mode == READWRITE_OPEN_CREATE) {
    return SQLITE_OPEN_READWRITE;
  }
  else {
    throw Error(string("Unknown open mode: ") + std::to_string(mode));
  }
}

template <typename Callback>
void SQLite3::execute(const string& query, Callback&& callback) {
  sqlite3_stmt* stmt;
  const char** pzTail = nullptr;
  int result = sqlite3_prepare_v2(db_, query.c_str(), query.length(), &stmt, pzTail);
  throwIfError(result);
  result = SQLITE_ROW;
  while (true) {
    result = sqlite3_step(stmt);
    if (result != SQLITE_ROW && result != SQLITE_DONE) {
      throw Error(getErrorMessage(result));
    }
    else if (result == SQLITE_ROW) {
      RowResult row_result(stmt);
      std::forward<Callback>(callback)(row_result);
    }
    else {
      break;
    }
  }
  result = sqlite3_finalize(stmt);
  throwIfError(result);
}

auto SQLite3::getErrorMessage(const int result) -> string {
  string msg = sqlite3_errmsg(db_);
  return msg;
}

bool SQLite3::hasErrorOccured(const int result, string* error_msg) {
  if (result != SQLITE_OK) {
    if (error_msg != nullptr) {
      *error_msg = sqlite3_errmsg(db_);
    }
    return true;
  }
  return false;
}

void SQLite3::throwIfError(const int result) {
  string error_msg;
  if (hasErrorOccured(result, &error_msg)) {
    throw Error(error_msg);
  }
}

SQLite3::Statement::Statement(SQLite3* sqlite_db, sqlite3_stmt* stmt)
    : sqlite_db_(sqlite_db), stmt_(stmt) {}

SQLite3::Statement::~Statement() {
  finish();
}

void SQLite3::Statement::finish() {
  if (stmt_ != nullptr) {
    int result = sqlite3_finalize(stmt_);
    stmt_ = nullptr;
    sqlite_db_->throwIfError(result);
  }
}

void SQLite3::Statement::bindNull(const int index) {
  int result = sqlite3_bind_null(stmt_, index);
  sqlite_db_->throwIfError(result);
}

void SQLite3::Statement::bindValue(const int index, const int value) {
  int result = sqlite3_bind_int(stmt_, index, value);
  sqlite_db_->throwIfError(result);
}

void SQLite3::Statement::bindValue(const int index, const size_t value) {
  int result = sqlite3_bind_int64(stmt_, index, value);
  sqlite_db_->throwIfError(result);
}

void SQLite3::Statement::bindValue64(const int index, const size_t value) {
  int result = sqlite3_bind_int64(stmt_, index, value);
  sqlite_db_->throwIfError(result);
}

void SQLite3::Statement::bindValue(const int index, const double value) {
  int result = sqlite3_bind_double(stmt_, index, value);
  sqlite_db_->throwIfError(result);
}

template <typename T>
void SQLite3::Statement::bindData(const int index, const std::vector<T>& data) {
  int result = sqlite3_bind_blob(stmt_, index, data.data(), data.size() * sizeof(T), SQLITE_STATIC);
  sqlite_db_->throwIfError(result);
}

SQLite3::RowResult::RowResult(sqlite3_stmt* stmt)
    : stmt_(stmt) {}

size_t SQLite3::RowResult::numColumns() const {
  return sqlite3_column_count(stmt_);
}

auto SQLite3::RowResult::getString(size_t col) const -> string {
  const unsigned char* text = sqlite3_column_text(stmt_, col);
  const string str = reinterpret_cast<const char*>(text);
  return str;
}

int SQLite3::RowResult::getInt(size_t col) const {
  return sqlite3_column_int(stmt_, col);
}

int64_t SQLite3::RowResult::getInt64(size_t col) const {
  return sqlite3_column_int64(stmt_, col);
}

double SQLite3::RowResult::getDouble(size_t col) const {
  return sqlite3_column_double(stmt_, col);
}

std::vector<uint8_t> SQLite3::RowResult::getRawData(size_t col) const {
  const int num_bytes = sqlite3_column_bytes(stmt_, col);
  std::vector<uint8_t> raw_data(num_bytes);
  const void* raw_data_ptr = sqlite3_column_blob(stmt_, col);
  std::memcpy(raw_data.data(), raw_data_ptr, num_bytes);
  return raw_data;
}

template <typename T>
std::vector<T> SQLite3::RowResult::getData(size_t col) const {
  const int num_bytes = sqlite3_column_bytes(stmt_, col);
  BH_ASSERT(num_bytes % sizeof(T) == 0);
  const int count = num_bytes / sizeof(T);
  std::vector<T> data(count);
  const void* raw_data_ptr = sqlite3_column_blob(stmt_, col);
  std::memcpy(data.data(), raw_data_ptr, num_bytes);
  return data;
}

template <typename T>
std::vector<T> SQLite3::RowResult::getData(size_t col, const int count) const {
  const int num_bytes = sqlite3_column_bytes(stmt_, col);
  BH_ASSERT(num_bytes >= count * sizeof(T));
  std::vector<T> data(count);
  const void* raw_data_ptr = sqlite3_column_blob(stmt_, col);
  std::memcpy(data.data(), raw_data_ptr, count * sizeof(T));
  return data;
}

}
