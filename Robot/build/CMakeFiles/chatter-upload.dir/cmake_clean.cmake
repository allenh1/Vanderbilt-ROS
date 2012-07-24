FILE(REMOVE_RECURSE
  "../chatter.eep"
  "../chatter.hex"
  "CMakeFiles/chatter-upload"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/chatter-upload.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
