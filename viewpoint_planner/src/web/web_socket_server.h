//==================================================
// websocket_server.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 14, 2017
//==================================================

#pragma once

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

QT_FORWARD_DECLARE_CLASS(QWebSocketServer)
QT_FORWARD_DECLARE_CLASS(QWebSocket)

class WebSocketServer : public QObject {

  Q_OBJECT

public:
  explicit WebSocketServer(quint16 port, QObject* parent = nullptr);
  ~WebSocketServer();

  void sendTextMessage(const std::string& msg);

  void sendBinaryMessage(const std::string& msg);

signals:
  void closed();

private Q_SLOTS:
  void onNewConnection();
  void processTextMessage(QString message);
  void processBinaryMessage(QByteArray message);
  void socketDisconnected();

private:
  QWebSocketServer* web_socket_server_;
  std::vector<QWebSocket*> clients_;

  std::string getPeerString(const QWebSocket* socket) const;
};
