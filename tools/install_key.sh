#!/bin/bash
mkdir .ssh 2>/dev/null
chmod 700 .ssh

if [ ! -f .ssh/authorized_keys ];
then
  cat /dev/null > .ssh/authorized_keys
fi
chmod 640 .ssh/authorized_keys

EXISTING=$(<.ssh/authorized_keys)
NEW=$(<key.pub)

if [[ "$EXISTING" =~ "$NEW" ]]; then
  echo "  > Key already installed. Skipping."
else
  echo "  > Installing new key."
  cat key.pub  >> .ssh/authorized_keys 
fi

rm ./key.pub
rm ./install_key.sh