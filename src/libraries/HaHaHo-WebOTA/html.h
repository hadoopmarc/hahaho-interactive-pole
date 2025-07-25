// Literal string
const char *indexHtml = R"literal(
  <!DOCTYPE html>
  <link rel='icon' href='/favicon.ico' sizes='any'>
  <body style='width:480px'>
    <h2>ESP Firmware Update</h2>
    <form method='POST' enctype='multipart/form-data' id='upload-form'>
      <input type='file' id='file' name='update'>
      <input type='submit' value='Update'>
    </form>
    <br>
    <div id='prg' style='width:0;color:white;text-align:center'>0%</div>
  </body>
  <script>
    var prg = document.getElementById('prg');
    var form = document.getElementById('upload-form');
    form.addEventListener('submit', el=>{
      prg.style.backgroundColor = 'blue';
      el.preventDefault();
      var data = new FormData(form);
      var req = new XMLHttpRequest();
      var fsize = document.getElementById('file').files[0].size;
      req.open('POST', '/update?size=' + fsize);
      req.upload.addEventListener('progress', p=>{
        let w = Math.round(p.loaded/p.total*100) + '%';
          if(p.lengthComputable){
             prg.innerHTML = w;
             prg.style.width = w;
          }
          if(w == '100%') prg.style.backgroundColor = 'black';
      });
      req.send(data);
     });
  </script>
)literal";

// Compressed gzip in C include file style
// listing was created using `xxd -i favicon.ico.gz`
// The favicon shows in a web browser tab as white radio waves on a red background
const char favicon_ico_gz[] = {
  0x1f, 0x8b, 0x08, 0x08, 0x13, 0xb6, 0xa5, 0x62, 0x00, 0x03, 0x66, 0x61, 0x76, 0x69, 0x63, 0x6f, 0x6e, 0x2e, 0x69, 0x63, 0x6f, 0x00, 0xa5, 0x53, 0x69, 0x48,
  0x54, 0x51, 0x14, 0x7e, 0x41, 0x99, 0x4b, 0xa0, 0x15, 0x24, 0xb4, 0x99, 0x8e, 0x1b, 0x25, 0xa8, 0x54, 0xb4, 0x19, 0x56, 0x62, 0x41, 0xab, 0x14, 0x45, 0x61,
  0x51, 0x62, 0x86, 0x69, 0x1a, 0xd4, 0x60, 0xda, 0xa2, 0x99, 0x4b, 0x69, 0x1b, 0x42, 0x26, 0x5a, 0x09, 0x59, 0xd8, 0xfe, 0xab, 0x88, 0xa0, 0x82, 0xe8, 0x4f,
  0x90, 0xa5, 0x14, 0x68, 0x0b, 0xe6, 0x2c, 0x6f, 0x9c, 0xed, 0xcd, 0x9b, 0xcd, 0x19, 0x9d, 0x99, 0x3b, 0xe3, 0xd7, 0x79, 0x6f, 0x34, 0xac, 0xe8, 0x57, 0xf7,
  0x72, 0x1e, 0x87, 0xb3, 0xbe, 0xf3, 0x9d, 0xef, 0x72, 0xdc, 0x24, 0xba, 0x51, 0x51, 0x1c, 0x7d, 0x17, 0x70, 0x85, 0x93, 0x39, 0x6e, 0x16, 0xc7, 0x71, 0xc9,
  0x24, 0x64, 0x22, 0x4b, 0xd0, 0x2e, 0x1f, 0xf2, 0xcd, 0x8c, 0x08, 0xca, 0xf8, 0x89, 0x48, 0x54, 0x7b, 0x49, 0xd8, 0xb8, 0x84, 0xc6, 0xab, 0xd9, 0xd4, 0x58,
  0x15, 0x8b, 0x4e, 0xd1, 0xb0, 0xa4, 0x0c, 0x9e, 0x2d, 0x5a, 0xa3, 0x63, 0x31, 0x4b, 0xb4, 0x6c, 0x9a, 0x22, 0x68, 0x0f, 0x9f, 0x10, 0x3b, 0x21, 0x17, 0x92,
  0x84, 0xc5, 0xa9, 0x90, 0xbc, 0x8a, 0x47, 0xe1, 0x31, 0x01, 0x95, 0xf5, 0x22, 0x6a, 0x1a, 0xac, 0x28, 0x3f, 0x2b, 0xa2, 0x8a, 0x74, 0xe5, 0x19, 0x0b, 0xb2,
  0xb7, 0xeb, 0x11, 0x49, 0x71, 0xe1, 0x09, 0xc1, 0xf8, 0x09, 0x75, 0x10, 0xae, 0x50, 0x61, 0xf3, 0x6e, 0x03, 0x2a, 0xeb, 0x44, 0xe4, 0xec, 0x31, 0xa0, 0xf9,
  0x86, 0x1d, 0xb7, 0x3b, 0x9d, 0x58, 0x9e, 0xa5, 0xc3, 0xb5, 0x1b, 0x0e, 0xd4, 0x5f, 0xb4, 0xe2, 0xa8, 0x52, 0xc0, 0xf1, 0x53, 0x16, 0xcc, 0x49, 0xd3, 0x8c,
  0xd7, 0x90, 0xf3, 0xa5, 0xbe, 0xdb, 0x72, 0x0d, 0x78, 0xff, 0x71, 0x04, 0x77, 0x1f, 0x3a, 0x91, 0x9e, 0xc9, 0xe3, 0xed, 0xbb, 0x61, 0xb8, 0x87, 0x03, 0x28,
  0x56, 0x9a, 0xc1, 0x0f, 0x32, 0xf4, 0x7d, 0xf5, 0x42, 0x91, 0xaa, 0xc1, 0xba, 0x6d, 0x7a, 0xc4, 0xaf, 0xe0, 0x11, 0x16, 0x1f, 0xcc, 0x0f, 0xa5, 0xb9, 0x16,
  0xae, 0xe6, 0x51, 0x7b, 0xc9, 0x8a, 0x17, 0xaf, 0xdd, 0x18, 0xf1, 0x04, 0xb0, 0x65, 0x97, 0x01, 0x1d, 0xf7, 0x9c, 0xf0, 0xfb, 0x03, 0x28, 0x2a, 0x13, 0xd0,
  0xfb, 0xcd, 0x8b, 0xfe, 0x1f, 0x5e, 0x1c, 0x38, 0x6c, 0xc2, 0xe3, 0xa7, 0x2e, 0x2c, 0xcd, 0x1e, 0x44, 0x98, 0x62, 0x2c, 0x9f, 0x30, 0x39, 0x52, 0x6e, 0x41,
  0xe6, 0x56, 0x3d, 0x16, 0x65, 0xf0, 0xd8, 0xb9, 0xd7, 0x88, 0x0d, 0x39, 0x7a, 0x0c, 0xa8, 0x7d, 0x30, 0x99, 0x19, 0x0e, 0x95, 0x9a, 0x60, 0xb6, 0x30, 0x74,
  0xf5, 0x8c, 0xa0, 0xbc, 0x52, 0xc0, 0x28, 0xdd, 0xcb, 0xd7, 0xed, 0x20, 0x2c, 0xe5, 0xfc, 0xd9, 0xa9, 0x1a, 0x56, 0xdd, 0x68, 0x45, 0x5c, 0xba, 0x16, 0xf9,
  0xa5, 0x66, 0xb4, 0xdd, 0x72, 0xa0, 0x7f, 0xc0, 0x0b, 0x50, 0x5c, 0x6b, 0xbb, 0x1d, 0xf5, 0x97, 0xad, 0xb2, 0x7e, 0xb3, 0xc3, 0x81, 0x96, 0x5b, 0x76, 0x59,
  0x3f, 0x5e, 0x69, 0xc1, 0xe4, 0x98, 0x60, 0x7e, 0x0a, 0xed, 0xe7, 0xd4, 0x39, 0x11, 0xe7, 0x29, 0x8e, 0xb1, 0x00, 0x46, 0x47, 0x47, 0x21, 0x50, 0xbf, 0x36,
  0xca, 0x2d, 0x2c, 0x31, 0x41, 0x6f, 0x64, 0xb0, 0x88, 0x0c, 0x05, 0xa4, 0x7f, 0xa1, 0x39, 0x2c, 0x56, 0x3f, 0x76, 0xec, 0x37, 0x22, 0x76, 0x99, 0x56, 0xc2,
  0x80, 0xa5, 0xad, 0xd5, 0xb1, 0x33, 0x84, 0x79, 0x17, 0x61, 0xe7, 0x72, 0xfb, 0x71, 0xe1, 0x8a, 0x0d, 0x07, 0x8b, 0x4d, 0xa8, 0x6d, 0x14, 0xa1, 0x1b, 0xf4,
  0xc1, 0x47, 0x35, 0xa5, 0xda, 0x4d, 0x2d, 0x36, 0xb9, 0xf7, 0xfd, 0x27, 0x4e, 0xd4, 0xd2, 0x2e, 0xf6, 0x15, 0x99, 0x11, 0x12, 0xa3, 0x62, 0x8a, 0xe5, 0x3c,
  0xab, 0x23, 0xff, 0xb3, 0x17, 0x2e, 0xb9, 0xf7, 0xf7, 0x7e, 0x2f, 0x0c, 0x26, 0x26, 0xeb, 0x16, 0x91, 0xea, 0x91, 0xaf, 0x82, 0x76, 0xef, 0x1c, 0xf2, 0x43,
  0xab, 0xf3, 0xc9, 0xb5, 0xbb, 0xba, 0x3d, 0x58, 0x4f, 0x18, 0x87, 0xc4, 0xaa, 0x59, 0x64, 0x82, 0x9a, 0x9d, 0xac, 0x15, 0x91, 0x9b, 0x6f, 0xc4, 0xab, 0x37,
  0xc3, 0x50, 0x6b, 0x7c, 0xe8, 0xfe, 0x34, 0x82, 0xf6, 0x3b, 0x0e, 0x14, 0x50, 0xec, 0xd5, 0x6b, 0x36, 0x38, 0x9c, 0x7e, 0xd8, 0x1c, 0x7e, 0x28, 0x4f, 0x0a,
  0xe8, 0x7c, 0xe4, 0xc4, 0xc7, 0x4f, 0x1e, 0xcc, 0x25, 0x0e, 0x84, 0x25, 0x04, 0x39, 0xb9, 0x25, 0xd7, 0x88, 0xfc, 0x12, 0x33, 0xa2, 0x93, 0xd5, 0xc8, 0x2b,
  0x32, 0xe1, 0xc4, 0x69, 0x01, 0xcd, 0xad, 0x76, 0xf4, 0xf6, 0x79, 0xe4, 0xff, 0xe0, 0xa9, 0xaf, 0xb2, 0x42, 0x40, 0x53, 0xb3, 0x0d, 0x43, 0x2e, 0xe2, 0x04,
  0xed, 0x34, 0x74, 0x0c, 0x7f, 0xe2, 0x11, 0x9b, 0x4e, 0x79, 0xe5, 0xd5, 0x22, 0xb2, 0x88, 0x1b, 0xcf, 0x5f, 0xba, 0xe5, 0x39, 0x25, 0x11, 0x09, 0xab, 0x07,
  0x34, 0xef, 0x41, 0xda, 0xfb, 0x3d, 0xea, 0xeb, 0x72, 0x07, 0x70, 0xf5, 0xba, 0x0d, 0x33, 0x92, 0xd5, 0xbf, 0xf3, 0x8f, 0xb8, 0x34, 0x7f, 0xb1, 0x16, 0x89,
  0x2b, 0x79, 0x64, 0x6c, 0x1a, 0x44, 0x53, 0x9b, 0x1d, 0x65, 0x55, 0x22, 0x76, 0x12, 0xce, 0x35, 0x0d, 0x22, 0x3e, 0xf4, 0x78, 0xd0, 0xfd, 0xd9, 0x43, 0x5c,
  0x14, 0x30, 0x23, 0xe9, 0x1f, 0xfc, 0x27, 0x9b, 0x54, 0x87, 0xf8, 0x28, 0xff, 0xdb, 0x94, 0xf9, 0x2a, 0x24, 0x10, 0x4f, 0xf3, 0x68, 0xae, 0x8d, 0xf4, 0x1e,
  0xe6, 0x11, 0x3f, 0x24, 0x7b, 0x78, 0xe2, 0xaf, 0x5c, 0xfc, 0xf1, 0x06, 0xff, 0x12, 0x69, 0xbf, 0x53, 0x68, 0x47, 0x21, 0x84, 0x91, 0xa4, 0x4f, 0xf4, 0x8d,
  0xbd, 0x63, 0x2f, 0xf7, 0x9f, 0xe7, 0x27, 0x40, 0xbb, 0x5a, 0x53, 0x7e, 0x04, 0x00, 0x00
};
const int favicon_ico_gz_len = 821;
