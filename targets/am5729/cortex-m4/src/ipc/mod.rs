//! ─────────────────────────────────────────────────────────────
//! AM5729 IPU1 — rpmsg IPC (M4 ↔ A15 통신)
//! ─────────────────────────────────────────────────────────────
//!
//! Linux remoteproc/rpmsg 프레임워크와 호환되는 M4측 IPC 구현.
//!
//! 통신 구조:
//!   A15 (Linux, pekko-rust)
//!     ↕ Mailbox5 인터럽트 + Shared Memory (VirtIO vring)
//!   M4 (이 펌웨어, RTIC)
//!
//! VirtIO/vring 프로토콜:
//!   - TX vring: M4 → A15 (M4가 쓰고, A15가 읽음)
//!   - RX vring: A15 → M4 (A15가 쓰고, M4가 읽음)
//!   - 각 vring은 descriptor table + available ring + used ring으로 구성
//!
//! 참고: TRM SPRUHZ6 — Chapter 17 (Mailbox), Linux rpmsg documentation

pub mod mailbox;
pub mod vring;
pub mod rpmsg;

pub use rpmsg::{RpmsgChannel, RpmsgEndpoint};
