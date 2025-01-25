use core::sync::atomic::{compiler_fence, Ordering};

use embassy_rp::dma::Channel;
use rp_pac::dma::vals::{DataSize, TreqSel};

pub fn init_ch<C: Channel>(
    ch: &C,
    from: *const u32,
    to: *mut u32,
    datasize: DataSize,
    chain_to: u8,
    quiet: bool,
) {
    let p = ch.regs();

    p.read_addr().write_value(from as u32);
    p.write_addr().write_value(to as u32);
    p.trans_count().write_value(1);

    compiler_fence(Ordering::SeqCst);

    p.ctrl_trig().write(|w| {
        w.set_treq_sel(TreqSel::PERMANENT);
        w.set_data_size(datasize);
        w.set_incr_read(false);
        w.set_incr_write(false);
        w.set_chain_to(chain_to);
        w.set_irq_quiet(quiet);
        w.set_en(true);
    });

    compiler_fence(Ordering::SeqCst);
}