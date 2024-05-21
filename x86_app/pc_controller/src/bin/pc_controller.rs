use std::borrow::Cow::{self, Borrowed, Owned};
use std::io::{Read, Write};
use std::time::Duration;

use clap::Parser;
use generic::atomi_proto::AtomiProto;
use generic::command_to_proto::parse_protocol;
use rustyline::{Cmd, CompletionType, Config, EditMode, Editor, KeyEvent};
use rustyline::completion::FilenameCompleter;
use rustyline::error::ReadlineError;
use rustyline::highlight::{Highlighter, MatchingBracketHighlighter};
use rustyline::hint::HistoryHinter;
use rustyline::validate::MatchingBracketValidator;
use rustyline_derive::{Completer, Helper, Hinter, Validator};
use pc_controller::find_serial_device;

#[derive(Helper, Completer, Hinter, Validator)]
struct MyHelper {
    #[rustyline(Completer)]
    completer: FilenameCompleter,
    highlighter: MatchingBracketHighlighter,
    #[rustyline(Validator)]
    validator: MatchingBracketValidator,
    #[rustyline(Hinter)]
    hinter: HistoryHinter,
    colored_prompt: String,
}

impl Highlighter for MyHelper {
    fn highlight_prompt<'b, 's: 'b, 'p: 'b>(
        &'s self,
        prompt: &'p str,
        default: bool,
    ) -> Cow<'b, str> {
        if default {
            Borrowed(&self.colored_prompt)
        } else {
            Borrowed(prompt)
        }
    }

    fn highlight_hint<'h>(&self, hint: &'h str) -> Cow<'h, str> {
        Owned("\x1b[1m".to_owned() + hint + "\x1b[m")
    }

    fn highlight<'l>(&self, line: &'l str, pos: usize) -> Cow<'l, str> {
        self.highlighter.highlight(line, pos)
    }

    fn highlight_char(&self, line: &str, pos: usize, forced: bool) -> bool {
        self.highlighter.highlight_char(line, pos, forced)
    }
}

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
#[command(next_line_help = true)]
struct Cli {
    #[arg(long)]
    #[arg(required_unless_present("probe"))]
    port: Option<String>,

    #[arg(long)]
    #[arg(required_unless_present("port"))]
    probe: Option<String>,

    #[arg(long, default_value_t = false)]
    with_len: bool,
}

// To debug rustyline:
// RUST_LOG=rustyline=debug cargo run --example example 2> debug.log
fn main() -> rustyline::Result<()> {
    env_logger::init();

    let cli = Cli::parse();
    println!("cli = {:?}", cli);

    let config = Config::builder()
        .history_ignore_space(true)
        .completion_type(CompletionType::List)
        .edit_mode(EditMode::Emacs)
        .build();
    let h = MyHelper {
        completer: FilenameCompleter::new(),
        highlighter: MatchingBracketHighlighter::new(),
        hinter: HistoryHinter::new(),
        colored_prompt: "".to_owned(),
        validator: MatchingBracketValidator::new(),
    };
    let mut rl = Editor::with_config(config)?;
    rl.set_helper(Some(h));
    rl.bind_sequence(KeyEvent::alt('n'), Cmd::HistorySearchForward);
    rl.bind_sequence(KeyEvent::alt('p'), Cmd::HistorySearchBackward);
    if rl.load_history("history.txt").is_err() {
        println!("No previous history.");
    }

    let port_name = match (cli.port, cli.probe) {
        (Some(port), _) => port,
        (_, Some(probe)) =>
            find_serial_device(probe.as_str())
                .expect(format!("Not found port with probe {}", probe).as_str()),
        _ => None.expect("No port or probe in your arguments"),
    };
    let mut port = serialport::new(port_name, 115_200)
        .timeout(Duration::from_secs(5))
        .open().expect("Failed to open port");

    let mut count = 1;
    loop {
        let p = format!("{count}> ");
        rl.helper_mut().expect("No helper").colored_prompt = format!("\x1b[1;32m{p}\x1b[0m");
        let readline = rl.readline(&p);
        match readline {
            Ok(line) => {
                rl.add_history_entry(line.as_str())?;
                let msg = parse_protocol(&line);
                if matches!(msg, AtomiProto::Unknown) {
                    println!("Unknown command '{}', ignore", line);
                    continue;
                }
                let data = postcard::to_vec::<AtomiProto, 8>(&msg).unwrap();

                if cli.with_len {
                    let l = data.len() as u8;
                    println!("Send data len: {}", l);
                    port.write(&[l]).expect("Errors in sending data len");
                }
                println!("Line: {}, protocol: {:?}, data = {:?}", line, msg, data);
                port.write(&data).expect(&format!("errors in sending request '{:?}'", data));

                let mut buf: Vec<u8> = vec![0u8; 64];
                let len = port.read(buf.as_mut_slice()).expect("errors in recving response");
                println!("Got response data: {:?}, len: {}", &buf[..len], len);
                let resp_data = if cli.with_len {
                    assert_eq!((len - 1) as u8, buf[0].into());
                    &buf[1..len]
                } else {
                    &buf[..len]
                };
                let resp = postcard::from_bytes::<AtomiProto>(resp_data);
                println!("Got response: ({len}) {:?}, msg: {:?}", &buf[..len], resp);
                port.flush().expect("Errors in flushing buffer for serial port");
            }
            Err(ReadlineError::Interrupted) => {
                println!("Interrupted");
                break;
            }
            Err(ReadlineError::Eof) => {
                println!("Encountered Eof");
                break;
            }
            Err(err) => {
                println!("Error: {err:?}");
                break;
            }
        }
        count += 1;
    }
    rl.append_history("history.txt")
}
