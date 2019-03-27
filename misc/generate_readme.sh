#!/usr/bin/env bash
cat readme.md > ../README.md

echo "" >> ../README.md
echo "" >> ../README.md
echo 'line count breakdown:' >> ../README.md
echo '```' >> ../README.md
./check_line_count.sh >> ../README.md
echo '```' >> ../README.md
