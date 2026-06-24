import SwiftUI

struct ContentView: View {
  @AppStorage("host") private var host = "192.168.0.10"
  @AppStorage("port") private var port = "7788"
  @AppStorage("token") private var token = ""
  @AppStorage("duration") private var duration = 1.5

  @State private var status = "Ready"
  private let client = UDPBlinkerClient()

  var body: some View {
    NavigationStack {
      Form {
        Section("comma") {
          TextField("Host", text: $host)
            .textInputAutocapitalization(.never)
            .keyboardType(.numbersAndPunctuation)
          TextField("Port", text: $port)
            .keyboardType(.numberPad)
          SecureField("Token, optional", text: $token)
            .textInputAutocapitalization(.never)
        }

        Section("Command") {
          HStack {
            Button("Left") {
              send(.left)
            }
            .buttonStyle(.borderedProminent)

            Button("Right") {
              send(.right)
            }
            .buttonStyle(.borderedProminent)
          }

          Button("Cancel") {
            send(.cancel)
          }
          .buttonStyle(.bordered)

          Button("Off") {
            send(.off)
          }
          .buttonStyle(.bordered)
        }

        Section("Duration") {
          Slider(value: $duration, in: 0.3...5.0, step: 0.1)
          Text(String(format: "%.1f seconds", duration))
        }

        Section("Status") {
          Text(status)
            .font(.footnote)
            .foregroundStyle(.secondary)
        }
      }
      .navigationTitle("Tesla Blinker")
    }
  }

  private func send(_ direction: BlinkerDirection) {
    guard let portNumber = UInt16(port) else {
      status = "Invalid port"
      return
    }

    Task {
      do {
        try await client.send(
          direction: direction,
          duration: duration,
          host: host,
          port: portNumber,
          token: token
        )
        status = "Sent \(direction.rawValue) to \(host):\(port)"
      } catch {
        status = "Send failed: \(error.localizedDescription)"
      }
    }
  }
}

#Preview {
  ContentView()
}
