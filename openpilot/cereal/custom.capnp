using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct DiagnosticRequest @0x81c2f05a394cf4af {
  sessionId @0 :UInt64;
  active @1 :Bool;
  obd @2 :Bool;
  route @3 :UInt32;
}

struct DiagnosticState @0xaedffd8f31e7b55d {
  sessionId @0 :UInt64;
  phase @1 :Phase;
  obd @2 :Bool;
  route @3 :UInt32;
  error @4 :Text;
  enum Phase {
    idle @0;
    preparing @1;
    scanning @2;
    restoring @3;
  }
}

struct DiagnosticSendcan @0xf35cc4560bbf6ec2 {
  sessionId @0 :UInt64;
  route @1 :UInt32;
  frames @2 :List(Frame);
  struct Frame {
    address @0 :UInt32;
    dat @1 :Data;
    src @2 :UInt8;
  }
}

struct DiagnosticCardAck @0xda96579883444c35 {
  sessionId @0 :UInt64;
  route @1 :UInt32;
}

struct DiagnosticControlsAck @0x80ae746ee2596b11 {
  sessionId @0 :UInt64;
  route @1 :UInt32;
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
