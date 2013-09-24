{
  "targets": [{ 
      "target_name": "alvar"
      , "sources": [ 
          "src/init.cc"
      
        , "src/Matrix.cc"
        , "src/Alvar_bind.cc"
        ]
      , 'libraries': [
          '<!@(pkg-config --libs opencv)',
          '-lalvar200',
          '-lalvarplatform200',
          '-L/media/DDLinux/alvar-2.0.0-src/src/',
           '-L/media/DDLinux/alvar-2.0.0-src/src/platform/'
        ]
      , 'cflags': [
            '<!@(pkg-config --cflags "opencv >= 2.3.1" )'
            , '-Wall'
            ,'-I/media/DDLinux/alvar-2.0.0-src/src'
          ]
      , 'cflags!' : [ '-fno-exceptions']
      , 'cflags_cc!': [ '-fno-rtti',  '-fno-exceptions']
      , "conditions": [
        ['OS=="mac"', {
          # cflags on OS X are stupid and have to be defined like this
          'xcode_settings': {
            'OTHER_CFLAGS': [
              '<!@(pkg-config --cflags opencv)'
            ]
            , "GCC_ENABLE_CPP_RTTI": "YES"
            , "GCC_ENABLE_CPP_EXCEPTIONS": "YES"
          }
        }]        
      
    ]
  }]
}

