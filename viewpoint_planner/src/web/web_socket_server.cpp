//==================================================
// web_socket_server.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 14, 2017
//==================================================

#include <iostream>

#include <QtWebSockets/qwebsocketserver.h>
#include <QtWebSockets/qwebsocket.h>
#include <QtCore/QDebug>

#include "web_socket_server.h"

using std::string;
using std::cout;
using std::endl;

QT_USE_NAMESPACE

WebSocketServer::WebSocketServer(quint16 port, QObject *parent /*= nullptr*/)
: QObject(parent),
  web_socket_server_(new QWebSocketServer(QStringLiteral("Quad3DR server"),
      QWebSocketServer::NonSecureMode, this)) {
  if (web_socket_server_->listen(QHostAddress::Any, port)) {
    cout << "WebSocketServer: Listening on port " << port << endl;
    connect(web_socket_server_, &QWebSocketServer::newConnection,
        this, &WebSocketServer::onNewConnection);
    connect(web_socket_server_, &QWebSocketServer::closed, this, &WebSocketServer::closed);
  }
}

WebSocketServer::~WebSocketServer()
{
  for (QWebSocket* client : clients_) {
    std::cout << "WebSocketServer: Closing client " << client << std::endl;
    client->close();
  }
  // Client sockets will be deleted by parent
  clients_.clear();
  web_socket_server_->close();
}

void WebSocketServer::sendTextMessage(const std::string& msg) {
//  cout << "WebSocketServer: Sending: " << msg << endl;
  for (QWebSocket* client : clients_) {
    client->sendTextMessage(QByteArray::fromStdString(msg));
  }
}

void WebSocketServer::sendBinaryMessage(const std::string& msg) {
  for (QWebSocket* client : clients_) {
    client->sendBinaryMessage(QByteArray::fromStdString(msg));
  }
}

void WebSocketServer::onNewConnection()
{
    QWebSocket* client = web_socket_server_->nextPendingConnection();

    connect(client, &QWebSocket::textMessageReceived, this, &WebSocketServer::processTextMessage);
    connect(client, &QWebSocket::binaryMessageReceived, this, &WebSocketServer::processBinaryMessage);
    connect(client, &QWebSocket::disconnected, this, &WebSocketServer::socketDisconnected);

    clients_.push_back(client);
    cout << "WebSocketServer: Connection " << clients_.size()
        << " from " << getPeerString(client) << endl;
}

void WebSocketServer::processTextMessage(QString message)
{
    QWebSocket* client = qobject_cast<QWebSocket*>(sender());
    cout << "WebSocketServer: Message received from " << getPeerString(client) << ":" << message.toStdString() << endl;
    if (client) {
      client->sendTextMessage(message);
    }
}

void WebSocketServer::processBinaryMessage(QByteArray message)
{
    QWebSocket* client = qobject_cast<QWebSocket*>(sender());
    cout << "WebSocketServer: Binary message received from " << getPeerString(client) << ":" << message.toStdString() << endl;
    if (client) {
      client->sendBinaryMessage(message);
    }
}

void WebSocketServer::socketDisconnected()
{
    QWebSocket* client = qobject_cast<QWebSocket*>(sender());
    cout << "WebSocketServer: Connection from " << getPeerString(client) << " closed" << endl;
    if (client) {
      clients_.erase(std::remove(clients_.begin(), clients_.end(), client));
      client->deleteLater();
    }
}

string WebSocketServer::getPeerString(const QWebSocket* socket) const {
  return socket->peerAddress().toString().toStdString() + std::string(":") + std::to_string(socket->peerPort());
}
