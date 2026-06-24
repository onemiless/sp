import Foundation
import Network

enum BlinkerDirection: String, Codable {
  case left
  case right
  case cancel
  case off
}

struct BlinkerCommand: Codable {
  let direction: BlinkerDirection
  let duration: Double
  let token: String?
}

final class UDPBlinkerClient {
  func send(direction: BlinkerDirection, duration: Double, host: String, port: UInt16, token: String) async throws {
    let endpointHost = NWEndpoint.Host(host)
    guard let endpointPort = NWEndpoint.Port(rawValue: port) else {
      throw URLError(.badURL)
    }

    let command = BlinkerCommand(
      direction: direction,
      duration: duration,
      token: token.isEmpty ? nil : token
    )
    let payload = try JSONEncoder().encode(command)
    let connection = NWConnection(host: endpointHost, port: endpointPort, using: .udp)

    try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Void, Error>) in
      var resumed = false

      func resumeOnce(_ result: Result<Void, Error>) {
        guard !resumed else { return }
        resumed = true
        connection.cancel()
        switch result {
        case .success:
          continuation.resume()
        case .failure(let error):
          continuation.resume(throwing: error)
        }
      }

      connection.stateUpdateHandler = { state in
        switch state {
        case .ready:
          connection.send(content: payload, completion: .contentProcessed { error in
            if let error {
              resumeOnce(.failure(error))
            } else {
              resumeOnce(.success(()))
            }
          })
        case .failed(let error):
          resumeOnce(.failure(error))
        default:
          break
        }
      }

      connection.start(queue: .global(qos: .userInitiated))
    }
  }
}
