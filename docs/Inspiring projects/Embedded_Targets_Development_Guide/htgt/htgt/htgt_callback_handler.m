function htgt_callback_handler(hDlg, hSrc)

  % Setup these options as desired and gray them out
  slConfigUISetVal(hDlg, hSrc, 'GenerateSampleERTMain', 'on');
  slConfigUISetEnabled(hDlg, hSrc, 'GenerateSampleERTMain', 0);

  slConfigUISetVal(hDlg, hSrc, 'GenerateMakefile', 'on');
  slConfigUISetEnabled(hDlg, hSrc, 'GenerateMakefile', 0);

  slConfigUISetVal(hDlg, hSrc, 'ERTCustomFileTemplate', 'htgt_file_process.tlc');
  slConfigUISetEnabled(hDlg, hSrc, 'ERTCustomFileTemplate', 0);

