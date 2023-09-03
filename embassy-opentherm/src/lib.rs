#![no_std]

#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

mod api;
pub mod pio_opentherm;

pub use api::*;

