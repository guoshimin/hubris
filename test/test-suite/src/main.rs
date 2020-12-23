//! Test suite.
//!
//! This task is driven by the `runner` to run test cases (defined below).
//!
//! Any test case that fails should indicate this by `panic!` (or equivalent,
//! like failing an `assert!`).
//!
//! # The assistant
//!
//! This test suite uses a second task, the assistant, to test IPC and
//! interactions. The assistant must be included in the image with the name
//! `assist`, but its ID is immaterial.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, Ordering};
use test_api::*;
use userlib::*;
use zerocopy::AsBytes;

/// Helper macro for building a list of functions with their names.
macro_rules! test_cases {
    ($($name:path),*) => {
        static TESTS: &[(&str, &(dyn Fn() + Send + Sync))] = &[
            $(
                (stringify!($name), &$name)
            ),*
        ];
    };
}

// Actual list of functions with their names.
test_cases! {
    test_send,
    test_recv_reply,
    test_fault_badmem,
    test_fault_stackoverflow,
    test_fault_execdata,
    test_fault_illop,
    test_fault_nullexec,
    test_fault_textoob,
    test_fault_stackoob,
    test_fault_buserror,
    test_fault_illinst,
    test_fault_divzero,
    test_panic,
    test_restart,
    test_restart_taskgen,
    test_borrow_info,
    test_borrow_read,
    test_borrow_write,
    test_supervisor_fault_notification,
    test_timer_advance,
    test_timer_notify,
    test_timer_notify_past
}

/// Tests that we can send a message to our assistant, and that the assistant
/// can reply. Technically this is also a test of RECV/REPLY on the assistant
/// side but hey.
fn test_send() {
    let assist = assist_task_id();
    let challenge = 0xDEADBEEF_u32;
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::JustReply as u16,
        &challenge.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    assert_eq!(response, !0xDEADBEEF);
}

/// Tests that we can receive a message from the assistant and reply.
fn test_recv_reply() {
    let assist = assist_task_id();

    // Ask the assistant to send us a message containing this challenge value.
    let challenge = 0xCAFE_F00Du32;
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::SendBack as u16,
        &challenge.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    // Don't actually care about the response in this case

    // Switch roles and wait for the message, blocking notifications.
    let rm = sys_recv_open(response.as_bytes_mut(), 0);
    assert_eq!(rm.sender, assist);
    assert_eq!(rm.operation, 42); // assistant always sends this

    // Check that we got the expected challenge back.
    assert_eq!(rm.message_len, 4);
    assert_eq!(response, challenge);

    // Check that the other message attributes seem legit.
    assert_eq!(rm.response_capacity, 4);
    assert_eq!(rm.lease_count, 0);

    // Send a recognizeable value in our reply; the assistant will record it.
    let reply_token = 0x1DE_u32;
    sys_reply(assist, 0, &reply_token.to_le_bytes());

    // Call back to the assistant and request a copy of our most recent reply.
    let (rc, len) = sys_send(
        assist,
        AssistOp::LastReply as u16,
        &challenge.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    assert_eq!(response, reply_token);
}

/// Helper routine to send a message to the assistant telling it to fault,
/// and then verifying that the fault caused a state change into the `Faulted`
/// state, returning the actual fault info.
fn test_fault(op: AssistOp, arg: u32) -> FaultInfo {
    let assist = assist_task_id();

    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        op as u16,
        &arg.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    // Don't actually care about the response in this case

    // Ask the kernel to report the assistant's state.
    let status = kipc::read_task_status(ASSIST as usize);

    match status {
        TaskState::Faulted {
            fault,
            original_state,
        } => {
            assert_eq!(original_state, SchedState::Runnable);
            fault
        }
        _ => {
            panic!("expected fault");
        }
    }
}

/// Tests a memory fault, which ensures that the address reporting is correct,
/// and that the MPU is on.
fn test_fault_badmem() {
    let bad_address = 5u32;
    let fault = test_fault(AssistOp::BadMemory, bad_address);

    assert_eq!(
        fault,
        FaultInfo::MemoryAccess {
            address: Some(bad_address),
            source: FaultSource::User,
        }
    );
}

fn test_fault_stackoverflow() {
    let fault = test_fault(AssistOp::StackOverflow, 0);

    match fault {
        FaultInfo::StackOverflow { .. } => {}
        _ => {
            panic!("expected StackOverflow; found {:?}", fault);
        }
    }
}

fn test_fault_execdata() {
    assert_eq!(test_fault(AssistOp::ExecData, 0), FaultInfo::IllegalText);
}

fn test_fault_illop() {
    let fault = test_fault(AssistOp::IllegalOperation, 0);

    match fault {
        FaultInfo::InvalidOperation { .. } => {}
        _ => {
            panic!("expected InvalidOperation; found {:?}", fault);
        }
    }
}

fn test_fault_nullexec() {
    assert_eq!(test_fault(AssistOp::BadExec, 0), FaultInfo::IllegalText);
}

fn test_fault_textoob() {
    let fault = test_fault(AssistOp::TextOutOfBounds, 0);

    match fault {
        FaultInfo::BusError { .. } | FaultInfo::MemoryAccess { .. } => {}
        _ => {
            panic!("expected BusFault or MemoryAccess; found {:?}", fault);
        }
    }
}

fn test_fault_stackoob() {
    let fault = test_fault(AssistOp::StackOutOfBounds, 0);
    match fault {
        FaultInfo::MemoryAccess { .. } => {}
        _ => {
            panic!("expected MemoryAccess; found {:?}", fault);
        }
    }
}

fn test_fault_buserror() {
    let fault = test_fault(AssistOp::BusError, 0);

    match fault {
        FaultInfo::BusError { .. } => {}
        _ => {
            panic!("expected BusFault; found {:?}", fault);
        }
    }
}

fn test_fault_illinst() {
    assert_eq!(
        test_fault(AssistOp::IllegalInstruction, 0),
        FaultInfo::IllegalInstruction
    );
}

/// Tests that division-by-zero results in a DivideByZero fault
fn test_fault_divzero() {
    assert_eq!(test_fault(AssistOp::DivZero, 0), FaultInfo::DivideByZero);
}

/// Tests that a `panic!` in a task is recorded as a fault.
fn test_panic() {
    let assist = assist_task_id();

    // Ask the assistant to panic.
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::Panic as u16,
        &0u32.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    // Don't actually care about the response in this case

    // Read status back from the kernel and check it.
    let status = kipc::read_task_status(ASSIST as usize);
    assert_eq!(
        status,
        TaskState::Faulted {
            fault: FaultInfo::Panic,
            original_state: SchedState::Runnable,
        },
    );
    restart_assistant();
}

/// Tests that task restart works as expected.
///
/// This is not a very thorough test right now.
fn test_restart() {
    let assist = assist_task_id();

    // First, store a value in state in the assistant task. More precisely, the
    // value is swapped for the previous contents, which should be zero.
    let value = 0xDEAD_F00D_u32;
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::Store as u16,
        &value.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);

    // Check that the old stored value (returned) is the bootup value
    assert_eq!(response, 0);

    // Read it back and replace it.
    let value2 = 0x1DE_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::Store as u16,
        &value2.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);

    assert_eq!(response, value);

    // Reboot the assistant and renew our task ID.
    restart_assistant();
    let assist = assist_task_id();

    // Swap values again.
    let (rc, len) = sys_send(
        assist,
        AssistOp::Store as u16,
        &value.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);

    // Confirm that the assistant lost our old value and returned to boot state.
    assert_eq!(response, 0);
}

/// Tests that when our task dies, we get an error code that consists of
/// the new generation in the lower bits.
fn test_restart_taskgen() {
    let assist = assist_task_id();

    // Ask the assistant to panic.
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::Panic as u16,
        &0u32.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);

    // Read status back from the kernel, check it, and bounce the assistant.
    let status = kipc::read_task_status(ASSIST as usize);
    assert_eq!(
        status,
        TaskState::Faulted {
            fault: FaultInfo::Panic,
            original_state: SchedState::Runnable,
        },
    );
    restart_assistant();

    // Now when we make another call with the old task, this should fail
    // with a hint as to our generation.
    let payload = 0xDEAD_F00Du32;
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::SendBack as u16,
        &payload.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );

    assert_eq!(rc & 0xffff_ff00, 0xffff_ff00);
    assert_eq!(len, 0);
    assert_ne!(assist.generation(), Generation::from((rc & 0xff) as u8));

    assert_eq!(
        assist_task_id().generation(),
        Generation::from((rc & 0xff) as u8)
    );
}

/// Tests that the basic `borrow_info` mechanics work by soliciting a
/// stereotypical loan from the assistant.
fn test_borrow_info() {
    let assist = assist_task_id();

    // Ask the assistant to call us back with two particularly shaped loans
    // (which are hardcoded in the assistant, not encoded here).
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::SendBackWithLoans as u16,
        &0u32.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    // Don't actually care about the response in this case

    // Receive...
    hl::recv_without_notification(
        response.as_bytes_mut(),
        |_op: u32, msg| -> Result<(), u32> {
            let (_msg, caller) = msg.fixed::<u32, u32>().unwrap();

            // Borrow 0 is expected to be 16 bytes long and R/W.
            let info0 = caller.borrow(0).info().unwrap();
            assert_eq!(
                info0.attributes,
                LeaseAttributes::READ | LeaseAttributes::WRITE
            );
            assert_eq!(info0.len, 16);

            // Borrow 1 is expected to be 5 bytes long and R/O.
            let info1 = caller.borrow(1).info().unwrap();
            assert_eq!(info1.attributes, LeaseAttributes::READ);
            assert_eq!(info1.len, 5);

            caller.reply(0);
            Ok(())
        },
    );
}

/// Tests that the `sys_borrow_read` facility is working on a basic level.
fn test_borrow_read() {
    let assist = assist_task_id();

    // Ask the assistant to call us back with two particularly shaped loans
    // (which are hardcoded in the assistant, not encoded here).
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::SendBackWithLoans as u16,
        &0u32.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    // Don't actually care about the response in this case

    // Receive:
    hl::recv_without_notification(
        response.as_bytes_mut(),
        |_op: u32, msg| -> Result<(), u32> {
            let (_msg, caller) = msg.fixed::<u32, u32>().unwrap();

            // Borrow #1 is the read-only one.

            let mut dest = [0; 5];
            // Read whole buffer
            caller.borrow(1).read_fully_at(0, &mut dest).unwrap();
            assert_eq!(&dest, b"hello");

            // Read just a part
            caller.borrow(1).read_fully_at(2, &mut dest[..3]).unwrap();
            assert_eq!(&dest[..3], b"llo");

            caller.reply(0);
            Ok(())
        },
    );
}

/// Tests that the `sys_borrow_write` facility is working on a basic level.
fn test_borrow_write() {
    let assist = assist_task_id();

    // Ask the assistant to call us back with two particularly shaped loans
    // (which are hardcoded in the assistant, not encoded here).
    let mut response = 0_u32;
    let (rc, len) = sys_send(
        assist,
        AssistOp::SendBackWithLoans as u16,
        &0u32.to_le_bytes(),
        response.as_bytes_mut(),
        &[],
    );
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    // Don't actually care about the response in this case

    hl::recv_without_notification(
        response.as_bytes_mut(),
        |_op: u32, msg| -> Result<(), u32> {
            let (_msg, caller) = msg.fixed::<u32, u32>().unwrap();

            // Borrow #0 is the read-write one.

            // Complete overwrite of buffer:
            caller.borrow(0).write_at(0, *b"hello, world(s)!").unwrap();

            let mut readback = [0; 16];
            caller.borrow(0).read_fully_at(0, &mut readback).unwrap();
            assert_eq!(&readback, b"hello, world(s)!");

            // Partial overwrite:
            caller.borrow(0).write_at(7, *b"llama").unwrap();

            caller.borrow(0).read_fully_at(0, &mut readback).unwrap();
            assert_eq!(&readback, b"hello, llama(s)!");

            caller.reply(0);
            Ok(())
        },
    );
}

/// Tests that faults in tasks are reported to the supervisor.
///
/// NOTE: this test depends on the supervisor fault mask, set in the test's
/// app.toml file, being `1`.
fn test_supervisor_fault_notification() {
    // First, clear the supervisor's stored notifications.
    read_runner_notifications();
    // Make sure they really cleared. Paranoia.
    assert_eq!(read_runner_notifications(), 0);

    // Now, ask the assistant to panic.
    {
        let assist = assist_task_id();
        let mut response = 0_u32;
        // Request a crash
        let (rc, len) = sys_send(
            assist,
            AssistOp::Panic as u16,
            &0u32.to_le_bytes(),
            response.as_bytes_mut(),
            &[],
        );
        assert_eq!(rc, 0);
        assert_eq!(len, 4);
        // Don't actually care about the response in this case
    }

    // Now, check the status.
    let n = read_runner_notifications();
    // The expected bitmask here is set in app.toml.
    assert_eq!(n, 1);
}

/// Tests that we can see the kernel timer advancing.
///
/// This test will fail by hanging. We can't set an iteration limit because who
/// knows how fast our computer is in relation to the tick rate?
fn test_timer_advance() {
    let initial_time = sys_get_timer().now;
    while sys_get_timer().now == initial_time {
        // doot doot
    }
}

/// Tests that we can set a timer in the future and receive a notification.
fn test_timer_notify() {
    const ARBITRARY_NOTIFICATION: u32 = 1 << 16;

    let start_time = sys_get_timer().now;
    // We'll arbitrarily set our deadline 2 ticks in the future.
    let deadline = start_time + 2;
    sys_set_timer(Some(deadline), ARBITRARY_NOTIFICATION);

    let rm = sys_recv_closed(&mut [], ARBITRARY_NOTIFICATION, TaskId::KERNEL)
        .unwrap();

    assert_eq!(rm.sender, TaskId::KERNEL);
    assert_eq!(rm.operation, ARBITRARY_NOTIFICATION);
    assert_eq!(rm.message_len, 0);
    assert_eq!(rm.response_capacity, 0);
    assert_eq!(rm.lease_count, 0);

    // In the interest of not making this test performance-sensitive, we merely
    // verify that the timer is at _or beyond_ our deadline.
    assert!(sys_get_timer().now >= deadline);
}

/// Tests that we can set a timer in the past and get immediate notification.
fn test_timer_notify_past() {
    const ARBITRARY_NOTIFICATION: u32 = 1 << 16;

    let start_time = sys_get_timer().now;
    let deadline = start_time;
    sys_set_timer(Some(deadline), ARBITRARY_NOTIFICATION);

    let rm = sys_recv_closed(&mut [], ARBITRARY_NOTIFICATION, TaskId::KERNEL)
        .unwrap();

    assert_eq!(rm.sender, TaskId::KERNEL);
    assert_eq!(rm.operation, ARBITRARY_NOTIFICATION);
    assert_eq!(rm.message_len, 0);
    assert_eq!(rm.response_capacity, 0);
    assert_eq!(rm.lease_count, 0);
}

///////////////////////////////////////////////////////////////////////////////
// Frameworky bits follow

/// Identity of our "assistant task" that we require in the image.
#[cfg(not(feature = "standalone"))]
const ASSIST: Task = Task::assist;

// For standalone mode -- this won't work, but then, neither will a task without
// a kernel.
#[cfg(feature = "standalone")]
const ASSIST: Task = Task::anonymous;

/// Tracks the current generation of the assistant task as we restart it.
static ASSIST_GEN: AtomicU8 = AtomicU8::new(0);

/// Gets the current expected `TaskId` for the assistant.
fn assist_task_id() -> TaskId {
    TaskId::for_index_and_gen(
        ASSIST as usize,
        Generation::from(ASSIST_GEN.load(Ordering::SeqCst)),
    )
}

/// Restarts the assistant task.
fn restart_assistant() {
    kipc::restart_task(ASSIST as usize, true);
    ASSIST_GEN.fetch_add(1, Ordering::SeqCst);
}

/// Contacts the runner task to read (and clear) its accumulated set of
/// notifications.
fn read_runner_notifications() -> u32 {
    let runner = TaskId::for_index_and_gen(0, Generation::default());
    let mut response = 0u32;
    let op = RunnerOp::ReadAndClearNotes as u16;
    let (rc, len) = sys_send(runner, op, &[], response.as_bytes_mut(), &[]);
    assert_eq!(rc, 0);
    assert_eq!(len, 4);
    response
}

/// Actual entry point.
#[export_name = "main"]
fn main() -> ! {
    // Work out the assistant generation. Restart it to ensure it's running
    // before we try talking to it. TODO: this is kind of gross, we need a way
    // to just ask.
    kipc::restart_task(ASSIST as usize, true);
    loop {
        let assist = assist_task_id();
        let challenge = 0xDEADBEEF_u32;
        let mut response = 0_u32;
        let (rc, _) = sys_send(
            assist,
            0,
            &challenge.to_le_bytes(),
            response.as_bytes_mut(),
            &[],
        );
        if rc == 0 {
            break;
        }
        ASSIST_GEN.fetch_add(1, Ordering::SeqCst);
    }

    let mut buffer = [0; 4];
    loop {
        hl::recv_without_notification(
            &mut buffer,
            |op, msg| -> Result<(), u32> {
                match op {
                    SuiteOp::GetCaseCount => {
                        let (_, caller) =
                            msg.fixed::<(), usize>().ok_or(2u32)?;
                        caller.reply(TESTS.len());
                    }
                    SuiteOp::GetCaseName => {
                        let (&idx, caller) =
                            msg.fixed::<usize, [u8; 64]>().ok_or(2u32)?;
                        let mut name_buf = [b' '; 64];
                        let name = TESTS[idx].0;
                        let name_len = name.len().min(64);
                        name_buf[..name_len]
                            .copy_from_slice(&name.as_bytes()[..name_len]);
                        caller.reply(name_buf);
                    }
                    SuiteOp::RunCase => {
                        let (&idx, caller) =
                            msg.fixed::<usize, ()>().ok_or(2u32)?;
                        let caller_tid = caller.task_id();
                        caller.reply(());

                        TESTS[idx].1();

                        let op = RunnerOp::TestComplete as u16;

                        // Call back with status.
                        let (rc, len) =
                            sys_send(caller_tid, op, &[], &mut [], &[]);
                        assert_eq!(rc, 0);
                        assert_eq!(len, 0);
                    }
                }
                Ok(())
            },
        )
    }
}